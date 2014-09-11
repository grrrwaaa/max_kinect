#include "MaxKinectBase.h"

#include <atlstr.h>

#include "NuiApi.h"

#pragma pack(push, 1)
    struct BGRA {
        unsigned char b, g, r, a;
    };
    struct ARGB {
        unsigned char a, r, g, b;
    };
    struct RGB {
        unsigned char r, g, b;
    };
	struct DepthPlayer {
		uint16_t d, p;
	};
#pragma pack(pop)

class t_kinect : public MaxKinectBase {
public:
	
	// Current Kinect
    INuiSensor* device;
	HANDLE colorStreamHandle;
	HANDLE depthStreamHandle;

	t_systhread capture_thread;	
	int capturing;

	t_kinect() {
		device = 0;
		colorStreamHandle = 0;
		capturing = 0;
		
		HRESULT result = NuiGetSensorCount(&device_count);
		if (result != S_OK) object_error(&ob, "failed to get sensor count");
	}

	~t_kinect() {
		close();
	}

	void open(t_symbol *s, long argc, t_atom * argv) {
		int index = 0;
		if (argc > 0) index = atom_getlong(argv);
		// TODO: support 'open serial'

		if (device) {
			object_warn(&ob, "device already opened");
			return;
		}
	
		INuiSensor* dev;
		HRESULT result = 0;
		result = NuiCreateSensorByIndex(index, &dev);
		if (result != S_OK) {
			object_error(&ob, "failed to create sensor");
			return;
		}
		
		result = dev->NuiStatus();
		switch(result) {
		case S_OK:
			break;
		case S_NUI_INITIALIZING:
			object_post(&ob, "the device is connected, but still initializing"); return;
		case E_NUI_NOTCONNECTED:
			object_error(&ob, "the device is not connected"); return;
		case E_NUI_NOTGENUINE:
			object_post(&ob, "the device is not a valid kinect"); break;
		case E_NUI_NOTSUPPORTED:
			object_post(&ob, "the device is not a supported model"); break;
		case E_NUI_INSUFFICIENTBANDWIDTH:
			object_error(&ob, "the device is connected to a hub without the necessary bandwidth requirements."); return;
		case E_NUI_NOTPOWERED:
			object_post(&ob, "the device is connected, but unpowered."); return;
		default:
			object_post(&ob, "the device has some unspecified error"); return;
		}
		
		device = dev;
		post("init device %p", device);

		long priority = 0; // maybe increase?
		if (systhread_create((method)&capture_threadfunc, this, 0, priority, 0, &capture_thread)) {
			object_error(&ob, "Failed to create capture thread.");
			capturing = 0;
			close();
			return;
		}
	}

	void run() {
		HRESULT result = 0;
		DWORD dwImageFrameFlags;
		DWORD initFlags = 0;

		CString id;

		initFlags |= NUI_INITIALIZE_FLAG_USES_COLOR;
		if (player) {
			initFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
		} else {
			initFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH;
		}
		//initFlags |= NUI_INITIALIZE_FLAG_USES_SKELETON;
		result = device->NuiInitialize(initFlags);
		if (result != S_OK) {
			object_error(&ob, "failed to initialize sensor");
			goto done;
		}
		
		object_post(&ob, "device initialized");

		dwImageFrameFlags = 0; 
		if (near_mode) dwImageFrameFlags |= NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;
		result = device->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR, //NUI_IMAGE_TYPE eImageType,
			NUI_IMAGE_RESOLUTION_640x480, // NUI_IMAGE_RESOLUTION eResolution,
			dwImageFrameFlags,
			2, //DWORD dwFrameLimit,
			0, 
			&colorStreamHandle);
		if (result != S_OK) {
			object_error(&ob, "failed to open stream");
			goto done;
		}
		
		object_post(&ob, "opened color stream");

		dwImageFrameFlags = 0; 
		dwImageFrameFlags |= NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES;
		if (near_mode) dwImageFrameFlags |= NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;
		NUI_IMAGE_TYPE eImageType = NUI_IMAGE_TYPE_DEPTH;
		if (player) eImageType = NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX;
		result = device->NuiImageStreamOpen(
			eImageType,
			NUI_IMAGE_RESOLUTION_640x480, // NUI_IMAGE_RESOLUTION eResolution,
			dwImageFrameFlags,
			2, //DWORD dwFrameLimit,
			0, 
			&depthStreamHandle);
		if (result != S_OK) {
			object_error(&ob, "failed to open stream");
			goto done;
		}
		
		object_post(&ob, "opened depth stream");

		id = (CString)(device->NuiUniqueId());

		object_post(&ob, "id %s", id);
		//object_post(&ob, "aid %s cid %s", (const char*)(_bstr_t(device->NuiAudioArrayId(), false)), (const char*)(_bstr_t(device->NuiDeviceConnectionId(), false)));

		capturing = 1;
		post("starting processing");
		while (capturing) {
			pollDepth();
			pollColor();
			//systhread_sleep(0);
		}
		post("finished processing");

	done:
		shutdown();
	}
	
	void shutdown() {
		if (device) {
			device->NuiShutdown();
			device->Release();
			device = 0;
		}
	}
	
	void close() {
		if (capturing) {
			capturing = 0;
			unsigned int ret;
			long result = systhread_join(capture_thread, &ret);
			post("thread closed");
		} else {
			shutdown();
		}
	}

	void pollDepth() {
		if (!device) return;

		HRESULT result;
		NUI_IMAGE_FRAME imageFrame;
		DWORD dwMillisecondsToWait = 200;

		result = device->NuiImageStreamGetNextFrame(depthStreamHandle, dwMillisecondsToWait, &imageFrame);
		if (result == E_NUI_FRAME_NO_DATA) {
			// timeout with no data. bail or continue?
			return;
		} else if (FAILED(result)) {
			switch(result) {
				case E_INVALIDARG:
				object_error(&ob, "arg stream error"); break;
				case E_POINTER:
				object_error(&ob, "pointer stream error"); break;
				default:
				object_error(&ob, "stream error"); break;
			}
			return;
		}
		INuiFrameTexture * imageTexture = NULL;
		BOOL bNearMode = near_mode;
		
		result = device->NuiImageFrameGetDepthImagePixelFrameTexture(depthStreamHandle, &imageFrame, &bNearMode, &imageTexture);
		//imageTexture = imageFrame.pFrameTexture;

		// got data; now turn it into jitter 
		if (!imageTexture) {
			post("no data");
			goto ReleaseFrame;
		}
		NUI_LOCKED_RECT LockedRect;
	
		// Lock the frame data so the Kinect knows not to modify it while we're reading it
		imageTexture->LockRect(0, &LockedRect, NULL, 0); 

		// Make sure we've received valid data
		if (LockedRect.Pitch != 0) {
			
			// convert to Jitter-friendly RGB layout:
			//const uint16_t * src = (const uint16_t *)LockedRect.pBits;
			NUI_DEPTH_IMAGE_PIXEL * src = (NUI_DEPTH_IMAGE_PIXEL *)LockedRect.pBits;
			uint32_t * dst = depth_back;
			int cells = DEPTH_HEIGHT * DEPTH_WIDTH;
			do {
				//*dst = (*src) & NUI_IMAGE_PLAYER_INDEX_MASK; // player ID
				//*dst = (*src) >> NUI_IMAGE_PLAYER_INDEX_SHIFT; // depth in mm
				*dst = src->depth;
				// src->playerIndex;
				dst++;
				src++;
			} while (--cells);
			new_depth_data = 1;
		}

		// We're done with the texture so unlock it
		imageTexture->UnlockRect(0);

		cloud_process();

	ReleaseFrame:
		// Release the frame
		device->NuiImageStreamReleaseFrame(depthStreamHandle, &imageFrame);

	}

	void pollColor() {
		int newframe = 0;
		if (!device) return;

		HRESULT result;
		NUI_IMAGE_FRAME imageFrame;
		DWORD dwMillisecondsToWait = 200;

		result = device->NuiImageStreamGetNextFrame(colorStreamHandle, dwMillisecondsToWait, &imageFrame);
		if (result == E_NUI_FRAME_NO_DATA) {
			// timeout with no data. bail or continue?
			return;
		} else if (FAILED(result)) {
			switch(result) {
				case E_INVALIDARG:
				object_error(&ob, "arg stream error"); break;
				case E_POINTER:
				object_error(&ob, "pointer stream error"); break;
				default:
				object_error(&ob, "stream error"); break;
			}
			//close();
			return;
		} 

		// got data; now turn it into jitter 
		//post("frame %d", imageFrame.dwFrameNumber);
		//outlet_int(outlet_msg, imageFrame.dwFrameNumber);
		INuiFrameTexture * imageTexture = imageFrame.pFrameTexture;
		NUI_LOCKED_RECT LockedRect;

		// Lock the frame data so the Kinect knows not to modify it while we're reading it
		imageTexture->LockRect(0, &LockedRect, NULL, 0); 

		// Make sure we've received valid data
		if (LockedRect.Pitch != 0) {
			//post("pitch %d size %d", LockedRect.Pitch, LockedRect.size);
			//static_cast<BYTE *>(LockedRect.pBits), LockedRect.size

			//sysmem_copyptr(LockedRect.pBits, rgb_back, LockedRect.size);

			// convert to Jitter-friendly RGB layout:
			const BGRA * src = (const BGRA *)LockedRect.pBits;
			RGB * dst = (RGB *)rgb_back;
			int cells = DEPTH_HEIGHT * DEPTH_WIDTH;
			do {
				dst->r = src->r;
				dst->g = src->g;
				dst->b = src->b;
				//dst->a = 0xFF;
				src++;
				dst++;
			} while (--cells);


			newframe = 1;
		}

		// We're done with the texture so unlock it
		imageTexture->UnlockRect(0);

//	ReleaseFrame:
		// Release the frame
		device->NuiImageStreamReleaseFrame(colorStreamHandle, &imageFrame);
		
		if (newframe) cloud_rgb_process();
		
		new_rgb_data = 1;
	}

	void led(int option) {
		object_warn(&ob, "LED not yet implemented for Windows");
	}

	void accel() {

		if (!device) return;
		
		Vector4 pReading;
		HRESULT result = device->NuiAccelerometerGetCurrentReading(&pReading);
		
		t_atom a[3];
		atom_setfloat(a+0, pReading.x);
		atom_setfloat(a+1, pReading.y);
		atom_setfloat(a+2, pReading.z);
		outlet_anything(outlet_msg, gensym("accel"), 3, a);
	}

	void getdevlist() {
//		t_atom a[8];
	}

	static void *capture_threadfunc(void *arg) {
		t_kinect *x = (t_kinect *)arg;
		x->run();
		systhread_exit(NULL);
		return NULL;
	}
};
