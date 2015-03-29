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
	
	Kinect() : KinectBase() {
		device = 0;
		// depth buffer doesn't use a jit_matrix, because uint16_t is not a Jitter type:
		depth_data = (uint16_t *)sysmem_newptr(KINECT_DEPTH_WIDTH*KINECT_DEPTH_HEIGHT * sizeof(uint16_t));
	}
	
	~Kinect() {
		
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
	
	static void rgb_callback(freenect_device *dev, void *pixels, uint32_t timestamp){
		Kinect *x = (Kinect *)freenect_get_user(dev);
		if(!x)return;
		
		//if (x->align_rgb_to_cloud) x->cloud_rgb_process();
		
		// nothing to do... the raw data is already there
		x->new_rgb_data = 1;
	}
	
	void depth_process() {
		// copy raw 16-bit depth data into 32-bit matrix for output:
		for (int i=0; i<KINECT_DEPTH_HEIGHT*KINECT_DEPTH_WIDTH; i++) {
			depth_back[i] = depth_data[i];
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
	
	CLASS_ATTR_LONG(maxclass, "align_rgb_to_cloud", 0, Kinect, align_rgb_to_cloud);
	CLASS_ATTR_STYLE(maxclass, "align_rgb_to_cloud", 0, "onoff");
	
	CLASS_ATTR_LONG(maxclass, "unique", 0, Kinect, unique);
	CLASS_ATTR_STYLE_LABEL(maxclass, "unique", 0, "onoff", "output frame only when new data is received");
	
	CLASS_ATTR_FLOAT_ARRAY(maxclass, "cloud_position", 0, Kinect, cloud_position, 3);
	CLASS_ATTR_FLOAT_ARRAY(maxclass, "cloud_quat", 0, Kinect, cloud_quat, 4);
	
	class_register(CLASS_BOX, maxclass); /* CLASS_NOBOX */
}
