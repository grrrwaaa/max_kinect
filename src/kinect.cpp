#include "kinect.h"

#include <new>
#include <string>

static t_class * maxclass = 0;

t_symbol * ps_floor_plane = 0;

t_symbol * ps_hip_center = 0;
t_symbol * ps_spine = 0;
t_symbol * ps_shoulder_center = 0;
t_symbol * ps_head = 0;

t_symbol * ps_shoulder_left = 0;
t_symbol * ps_elbow_left = 0;
t_symbol * ps_wrist_left = 0;
t_symbol * ps_hand_left = 0;

t_symbol * ps_shoulder_right = 0;
t_symbol * ps_elbow_right = 0;
t_symbol * ps_wrist_right = 0;
t_symbol * ps_hand_right = 0;

t_symbol * ps_hip_left = 0;
t_symbol * ps_knee_left = 0;
t_symbol * ps_ankle_left = 0;
t_symbol * ps_foot_left = 0;

t_symbol * ps_hip_right = 0;
t_symbol * ps_knee_right = 0;
t_symbol * ps_ankle_right = 0;
t_symbol * ps_foot_right = 0;

#ifdef __APPLE__
#include "CoreFoundation/CoreFoundation.h"

extern "C" {
	#include "libfreenect.h"
}

static freenect_context * f_ctx = NULL;
static t_systhread capture_thread;
static int capturing = 0;
static int opendevices = 0;

class Kinect : public KinectBase {
public:
	
	// freenect:
	freenect_device  *device;
	
	// internal data:
	uint16_t *	depth_data;
	glm::i8vec3 * rgb_data;
	
	Kinect() : KinectBase() {
		device = 0;
		// depth buffer doesn't use a jit_matrix, because uint16_t is not a Jitter type:
		depth_data = (uint16_t *)sysmem_newptr(KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT * sizeof(uint16_t));
		rgb_data = (glm::i8vec3 *)sysmem_newptr(KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT * sizeof(glm::i8vec3));

	}
	
	~Kinect() {
		close();
		sysmem_freeptr(depth_data);
		sysmem_freeptr(rgb_data);
	}
	
	void getdevlist() {
		t_atom a[8];
		int i=0;
		struct freenect_device_attributes* attribute_list;
		struct freenect_device_attributes* attribute;
		
		// list devices:
		if (!f_ctx) return;
		
		int num_devices = freenect_list_device_attributes(f_ctx, &attribute_list);
		
		attribute = attribute_list;
		while (attribute) {
			atom_setsym(a+i, gensym(attribute->camera_serial));
			attribute = attribute->next;
		}
		outlet_anything(outlet_msg, gensym("devlist"), num_devices, a);
		
		freenect_free_device_attributes(attribute_list);
		
		//post("supported flags: %d", freenect_supported_subdevices());
	}
	
	static void log_cb(freenect_context *dev, freenect_loglevel level, const char *msg) {
		object_post(NULL, msg);
	}
	
	void open(t_symbol *s, long argc, t_atom *argv) {
		
		if (device){
			object_post(&ob, "A device is already open.");
			return;
		}
		
		// mark one additional device:
		opendevices++;
		
		if(!f_ctx){
			long priority = 0; // maybe increase?
			if (systhread_create((method)&capture_threadfunc, this, 0, priority, 0, &capture_thread)) {
				object_error(&ob, "Failed to create capture thread.");
				capturing = 0;
				return;
			}
			while(!f_ctx){
				systhread_sleep(100);
			}
		}
		
		int ndevices = freenect_num_devices(f_ctx);
		if(!ndevices){
			object_post(&ob, "Could not find any connected kinect device. Are you sure the power cord is plugged-in?");
			capturing = 0;
			return;
		}
		
		if (argc > 0 && atom_gettype(argv) == A_SYM) {
			const char * serial = atom_getsym(argv)->s_name;
			object_post(&ob, "opening device %s", serial);
			if (freenect_open_device_by_camera_serial(f_ctx, &device, serial) < 0) {
				object_error(&ob, "failed to open device %s", serial);
				device = NULL;
			}
		} else {
			int devidx = 0;
			if (argc > 0 && atom_gettype(argv) == A_LONG) devidx = atom_getlong(argv);
			
			object_post(&ob, "opening device %d", devidx);
			if (freenect_open_device(f_ctx, &device, devidx) < 0) {
				object_error(&ob, "failed to open device %d", devidx);
				device = NULL;
			}
		}
		
		if (!device) {
			// failed to create device:
			opendevices--;
			return;
		}
		
		freenect_set_user(device, this);
		freenect_set_depth_callback(device, depth_callback);
		freenect_set_video_callback(device, rgb_callback);
		
		freenect_set_video_buffer(device, rgb_back);
		freenect_set_depth_buffer(device, depth_data);
		
		freenect_set_video_mode(device, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
		freenect_set_depth_mode(device, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
		
		freenect_set_led(device,LED_RED);
		
		freenect_start_depth(device);
		if (use_rgb) freenect_start_video(device);
	}

	void close() {
		if(!device) return;
		
		freenect_set_led(device,LED_BLINK_GREEN);
		freenect_close_device(device);
		device = NULL;
		
		// mark one less device running:
		opendevices--;
	}
	
	static void *capture_threadfunc(void *arg) {
		Kinect *x = (Kinect *)arg;
		
		// create the freenect context:
		freenect_context *context;
		if(!f_ctx){
			if (freenect_init(&context, NULL) < 0) {
				object_error(NULL, "freenect_init() failed");
				goto out;
			}
		}
		f_ctx = context;
		
		freenect_set_log_callback(f_ctx, log_cb);
		//		FREENECT_LOG_FATAL = 0,     /**< Log for crashing/non-recoverable errors */
		//		FREENECT_LOG_ERROR,         /**< Log for major errors */
		//		FREENECT_LOG_WARNING,       /**< Log for warning messages */
		//		FREENECT_LOG_NOTICE,        /**< Log for important messages */
		//		FREENECT_LOG_INFO,          /**< Log for normal messages */
		//		FREENECT_LOG_DEBUG,         /**< Log for useful development messages */
		//		FREENECT_LOG_SPEW,          /**< Log for slightly less useful messages */
		//		FREENECT_LOG_FLOOD,         /**< Log EVERYTHING. May slow performance. */
		freenect_set_log_level(f_ctx, FREENECT_LOG_WARNING);
		
		capturing = 1;
		object_post(NULL, "freenect starting processing");
		while (capturing > 0) {
			int err = freenect_process_events(f_ctx);
			//int err = freenect_process_events_timeout(f_ctx);
			if(err < 0){
				object_error(NULL, "Freenect could not process events.");
				break;
			}
			systhread_sleep(0);
		}
		object_post(NULL, "freenect finished processing");
		
	out:
		freenect_shutdown(f_ctx);
		f_ctx = NULL;
		systhread_exit(NULL);
		return NULL;
	}
	
	void rgb_process(glm::i8vec3 *pixels) {
		if (mirror) makeMirror(pixels);
		
		//if (x->align_rgb_to_cloud) x->cloud_rgb_process();
		
		new_rgb_data = 1;
	}
	
	static void rgb_callback(freenect_device *dev, void *pixels, uint32_t timestamp){
		Kinect *x = (Kinect *)freenect_get_user(dev);
		if(!x)return;
		
		x->rgb_process((glm::i8vec3 *)pixels);
	}
	
	void depth_process() {
		if (mirror) {
			for (int i=0; i<KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; i += KINECT_DEPTH_WIDTH) {
				uint32_t * dst = depth_back + i;
				uint16_t * src = depth_data + i + (KINECT_DEPTH_WIDTH-1);
				int cells = KINECT_DEPTH_WIDTH;
				do {
					*dst++ = *src++;
				} while (--cells);
			}
		} else {
			uint32_t * dst = depth_back;
			uint16_t * src = depth_data;
			int cells = KINECT_DEPTH_HEIGHT * KINECT_DEPTH_WIDTH;
			do {
				*dst++ = *src++;
			} while (--cells);
		}
		
		new_depth_data = 1;
		
		//cloud_process();	// generate point cloud
	}
	
	static void depth_callback(freenect_device *dev, void *pixels, uint32_t timestamp){
		Kinect *x = (Kinect *)freenect_get_user(dev);
		if(!x)return;
		
		x->depth_process();
	}
	
	
};

#else

// for vs2008:
#ifndef static_assert
	#define static_assert()
#endif

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

static int device_count;

class Kinect : public KinectBase {
public:

	INuiSensor* device;
	INuiCoordinateMapper * mapper;
	HANDLE colorStreamHandle;
	HANDLE depthStreamHandle;

	NUI_SKELETON_FRAME skeleton_back;

	t_systhread capture_thread;	
	int capturing;

	NUI_COLOR_IMAGE_POINT * imagepoints;
	Vector4 * skeletonpoints;

	Kinect() : KinectBase() {
		device = 0;
		mapper = 0;
		colorStreamHandle = 0;
		depthStreamHandle = 0;
		capturing = 0;

		imagepoints = (NUI_COLOR_IMAGE_POINT *)sysmem_newptr(KINECT_DEPTH_CELLS * sizeof(NUI_COLOR_IMAGE_POINT));
		skeletonpoints = (Vector4 *)sysmem_newptr(KINECT_DEPTH_CELLS * sizeof(Vector4));
	}
	
	~Kinect() {
		close();

		sysmem_freeptr(imagepoints);
		sysmem_freeptr(skeletonpoints);
	}

	
	void open(t_symbol *s, long argc, t_atom * argv) {

		HRESULT result = NuiGetSensorCount(&device_count);
		if (result != S_OK) { 
			object_error(&ob, "failed to get sensor count");
			return;
		}

		int index = 0;
		if (argc > 0) index = atom_getlong(argv);
		// TODO: support 'open serial'

		if (device) {
			object_warn(&ob, "device already opened");
			return;
		}
	
		INuiSensor* dev;
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
	
	void shutdown() {
		if (device) {
			device->NuiShutdown();
			device->Release();
			device = 0;
		}
	}

	void run() {
		HRESULT result = 0;
		DWORD dwImageFrameFlags;
		DWORD initFlags = 0;

		//CString id;

		initFlags |= NUI_INITIALIZE_FLAG_USES_COLOR;
//		if (player) {
//			initFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX;
//		} else {
			initFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH;
//		}
//		if (skeleton) {
//			initFlags |= NUI_INITIALIZE_FLAG_USES_SKELETON;
//		}
		result = device->NuiInitialize(initFlags);
		if (result != S_OK) {
			object_error(&ob, "failed to initialize sensor");
			goto done;
		}

//		if (skeleton) {
//			if (seated) {
//				NuiSkeletonTrackingEnable(NULL, NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);
//			} else {
//				NuiSkeletonTrackingEnable(NULL, 0);
//			}
//		}
		
		object_post(&ob, "device initialized");

		dwImageFrameFlags = 0; 
//		if (near_mode) dwImageFrameFlags |= NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;
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
//		if (near_mode) dwImageFrameFlags |= NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;
		NUI_IMAGE_TYPE eImageType = NUI_IMAGE_TYPE_DEPTH;
//		if (player) eImageType = NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX;
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

		//id = (CString)(device->NuiUniqueId());

		//object_post(&ob, "id %s", id);
		//object_post(&ob, "aid %s cid %s", (const char*)(_bstr_t(device->NuiAudioArrayId(), false)), (const char*)(_bstr_t(device->NuiDeviceConnectionId(), false)));

		// get coordinate mapper:
		device->NuiGetCoordinateMapper(&mapper);

		capturing = 1;
		post("starting processing");
		while (capturing) {
			pollDepth();
			pollColor();
//			if (skeleton) pollSkeleton();
			//systhread_sleep(0);
		}
		post("finished processing");

	done:
		shutdown();
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
			if (mirror) {
				uint32_t * dst = depth_back;
				NUI_DEPTH_IMAGE_PIXEL * src = (NUI_DEPTH_IMAGE_PIXEL *)LockedRect.pBits;
				int cells = KINECT_DEPTH_HEIGHT * KINECT_DEPTH_WIDTH;
				do {
					*dst++ = src->depth;
					//					*dstp++ = (char)src->playerIndex;
					src++;
				} while (--cells);
			} else {
				for (int i=0; i<KINECT_DEPTH_CELLS; i += KINECT_DEPTH_WIDTH) {
					uint32_t * dst = depth_back + i;
					NUI_DEPTH_IMAGE_PIXEL * src = ((NUI_DEPTH_IMAGE_PIXEL *)LockedRect.pBits) + i + (KINECT_DEPTH_WIDTH-1);
					int cells = KINECT_DEPTH_WIDTH;
					do {
						*dst++ = src->depth;
//						*dstp++ = (char)src->playerIndex;
						src--;
					} while (--cells);
				}
			}

//			// convert to Jitter-friendly RGB layout:
//			//const uint16_t * src = (const uint16_t *)LockedRect.pBits;
//			NUI_DEPTH_IMAGE_PIXEL * src = (NUI_DEPTH_IMAGE_PIXEL *)LockedRect.pBits;
//			uint32_t * dst = depth_back;
//			//			char * dstp = player_back;
//			if (mirror) {
//				int cells = KINECT_DEPTH_HEIGHT * KINECT_DEPTH_WIDTH;
//				do {
//					*dst++ = src->depth;
////					*dstp++ = (char)src->playerIndex;
//					src++;
//				} while (--cells);
//			} else {
//				for (int i=0; i<KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; i += KINECT_DEPTH_WIDTH) {
//					for (int x=0; x<KINECT_DEPTH_WIDTH; ++x) {
//						int i1 = i+x;
//						int i0 = i+(KINECT_DEPTH_WIDTH-1)-x;
//						//*dst = (*src) & NUI_IMAGE_PLAYER_INDEX_MASK; // player ID
//						//*dst = (*src) >> NUI_IMAGE_PLAYER_INDEX_SHIFT; // depth in mm
//						dst[i1] = src[i0].depth;
//		//				*dstp++ = (char)src->playerIndex;
//						//src++;
//					}
//				} 
//			}

			new_depth_data = 1;

			// generate texcoords;
			if (mapper->MapDepthFrameToColorFrame(
				NUI_IMAGE_RESOLUTION_640x480,
				KINECT_DEPTH_CELLS, (NUI_DEPTH_IMAGE_PIXEL *)LockedRect.pBits,
				NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480,
				KINECT_DEPTH_CELLS, imagepoints)) {
				object_warn(&ob, "failed to generate texcoords");
			} else {
				// convert imagepoints to texcoords:
				int cells = KINECT_DEPTH_CELLS;
				if (mirror) {
					for (int i=0; i<cells; i++) {
						glm::vec2& dst = texcoord_back[i];
						const NUI_COLOR_IMAGE_POINT& src = imagepoints[i];
						dst.x = src.x * (1.f/KINECT_DEPTH_WIDTH);
						dst.y = 1. - src.y * (1.f/KINECT_DEPTH_HEIGHT);
					} 
				} else {
					for (int i=0; i<cells; i++) {
						glm::vec2& dst = texcoord_back[i];
						const NUI_COLOR_IMAGE_POINT& src = imagepoints[i];
						dst.x = 1. - src.x * (1.f/KINECT_DEPTH_WIDTH);
						dst.y = 1. - src.y * (1.f/KINECT_DEPTH_HEIGHT);
					} 
				}
			}
			
			// let the Kinect SDK convert our depth frame to points in "skeleton space"
			if (mapper->MapDepthFrameToSkeletonFrame(NUI_IMAGE_RESOLUTION_640x480,
				KINECT_DEPTH_CELLS, (NUI_DEPTH_IMAGE_PIXEL *)LockedRect.pBits,
				KINECT_DEPTH_CELLS, skeletonpoints)) {
				object_warn(&ob, "failed to map depth to cloud");
			} else {

				// convert to proper vec3:
				int cells = KINECT_DEPTH_CELLS;
				for (int i=0; i<cells; i++) {
					glm::vec3& dst = cloud_back[i];
					const Vector4& src = skeletonpoints[i];
					// scale appropriately:
					float div = 1.f/src.w;
					// also rotate 180 around y
					dst.x = -src.x * div;
					dst.y =  src.y * div;
					dst.z = -src.z * div;
				} 
				
				new_cloud_data = 1;
			}
		}

		

		// We're done with the texture so unlock it
		imageTexture->UnlockRect(0);

//		cloud_process();
//		local_cloud_process();


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
			const BGRA * src = (const BGRA *)LockedRect.pBits;
			RGB * dst = (RGB *)rgb_back;
			if (mirror) {
				int cells = KINECT_DEPTH_HEIGHT * KINECT_DEPTH_WIDTH;
				do {
					dst->r = src->r;
					dst->g = src->g;
					dst->b = src->b;
					//dst->a = 0xFF;
					src++;
					dst++;
				} while (--cells);
			} else {
				for (int i=0; i<KINECT_DEPTH_WIDTH * KINECT_DEPTH_HEIGHT; i += KINECT_DEPTH_WIDTH) {
					for (int x=0; x<KINECT_DEPTH_WIDTH; ++x) {
						const BGRA& s = src[ i+(KINECT_DEPTH_WIDTH-1)-x];
						RGB& d = dst[i+x];
						d.r = s.r;
						d.g = s.g;
						d.b = s.b;
					}
				} 
			}


			newframe = 1;
		}

		// We're done with the texture so unlock it
		imageTexture->UnlockRect(0);

//	ReleaseFrame:
		// Release the frame
		device->NuiImageStreamReleaseFrame(colorStreamHandle, &imageFrame);
		
//		if (newframe) cloud_rgb_process();
		
		new_rgb_data = 1;
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

	void getdevlist() {}

	static void *capture_threadfunc(void *arg) {
		Kinect *x = (Kinect *)arg;
		x->run();
		systhread_exit(NULL);
		return NULL;
	}
};

#endif	// #ifdef __APPLE__

void kinect_bang(Kinect * x) { x->bang(); }

void kinect_getdevlist(Kinect *x) {
	x->getdevlist();
}

void kinect_doopen(Kinect *x, t_symbol *s, long argc, t_atom *argv) {
	x->open(s, argc, argv);
}

void kinect_open(Kinect *x, t_symbol *s, long argc, t_atom *argv) {
	//x->open(s, argc, argv);
	defer_low(x, (method)kinect_doopen, s, argc, argv);
}

void kinect_close(Kinect *x) {
	x->close();
}

void kinect_assist(Kinect *x, void *b, long m, long a, char *s)
{
	if (m == ASSIST_INLET) { // inlet
		if (a == 0) {
			sprintf(s, "messages in, bang to report orientation");
		} else {
			sprintf(s, "I am inlet %ld", a);
		}
	} else {	// outlet
		if (a == 0) {
			sprintf(s, "3D point cloud (matrix)");
		} else if (a == 1) {
			sprintf(s, "depth data (matrix)");
		} else if (a == 2) {
			sprintf(s, "RGB data (matrix)");
		} else if (a == 3) {
			sprintf(s, "player ID (matrix)");
		} else {
			sprintf(s, "messages out");
		}
	}
}

void *kinect_new(t_symbol *s, long argc, t_atom *argv) {
	Kinect *x = NULL;
	if ((x = (Kinect *)object_alloc(maxclass))) {
		
		// initialize in-place:
		x = new ((void *)x) Kinect();
		
		// apply attrs:
		attr_args_process(x, argc, argv);
	}
	return (x);
}

void kinect_free(Kinect *x) {
	x->~Kinect();
}

extern "C"
void C74_EXPORT ext_main(void* unused) {
	
	common_symbols_init();
	
	ps_floor_plane = gensym("floor_plane");
	
	ps_hip_center = gensym("hip_center");
	ps_spine = gensym("spine");
	ps_shoulder_center = gensym("shoulder_center");
	ps_head = gensym("head");
	
	ps_shoulder_left = gensym("shoulder_left");
	ps_elbow_left = gensym("elbow_left");
	ps_wrist_left = gensym("wrist_left");
	ps_hand_left = gensym("hand_left");
	
	ps_shoulder_right = gensym("shoulder_right");
	ps_elbow_right = gensym("elbow_right");
	ps_wrist_right = gensym("wrist_right");
	ps_hand_right = gensym("hand_right");
	
	ps_hip_left = gensym("hip_left");
	ps_knee_left = gensym("knee_left");
	ps_ankle_left = gensym("ankle_left");
	ps_foot_left = gensym("foot_left");
	
	ps_hip_right = gensym("hip_right");
	ps_knee_right = gensym("knee_right");
	ps_ankle_right = gensym("ankle_right");
	ps_foot_right = gensym("foot_right");
	
	maxclass = class_new("kinect", (method)kinect_new, (method)kinect_free, (long)sizeof(Kinect), 0L, A_GIMME, 0);
	
	class_addmethod(maxclass, (method)kinect_assist, "assist", A_CANT, 0);
	
	class_addmethod(maxclass, (method)kinect_bang, "bang", 0);
	class_addmethod(maxclass, (method)kinect_getdevlist, "getdevlist", 0);
	class_addmethod(maxclass, (method)kinect_open, "open", A_GIMME, 0);
	class_addmethod(maxclass, (method)kinect_close, "close", 0);

	
	CLASS_ATTR_LONG(maxclass, "use_rgb", 0, Kinect, use_rgb);
	CLASS_ATTR_STYLE(maxclass, "use_rgb", 0, "onoff");

	CLASS_ATTR_LONG(maxclass, "mirror", 0, Kinect, mirror);
	CLASS_ATTR_STYLE(maxclass, "mirror", 0, "onoff");
	
	CLASS_ATTR_LONG(maxclass, "align_rgb_to_cloud", 0, Kinect, align_rgb_to_cloud);
	CLASS_ATTR_STYLE(maxclass, "align_rgb_to_cloud", 0, "onoff");
	
	CLASS_ATTR_LONG(maxclass, "unique", 0, Kinect, unique);
	CLASS_ATTR_STYLE_LABEL(maxclass, "unique", 0, "onoff", "output frame only when new data is received");
	
	CLASS_ATTR_FLOAT_ARRAY(maxclass, "cloud_position", 0, Kinect, cloud_position, 3);
	CLASS_ATTR_FLOAT_ARRAY(maxclass, "cloud_quat", 0, Kinect, cloud_quat, 4);
	
	class_register(CLASS_BOX, maxclass); /* CLASS_NOBOX */
}
