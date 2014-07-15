#include "MaxKinectBase.h"

extern "C" {
	#include "libfreenect.h"
}

freenect_context * f_ctx = NULL;
t_systhread capture_thread;	
int capturing = 0;

class t_kinect : public MaxKinectBase {
public:

	// freenect:
	freenect_device  *device;
	
	// internal data:
	uint16_t *	depth_data;
		
	t_kinect() {	
		device = 0;
			
		// depth buffer doesn't use a jit_matrix, because uint16_t is not a Jitter type:
		depth_data = (uint16_t *)sysmem_newptr(DEPTH_WIDTH*DEPTH_HEIGHT * sizeof(uint16_t));
	}

	~t_kinect() {
		close();
		
		sysmem_freeptr(depth_data);
	}
	
	void getdevlist() {
		t_atom a[8];
		int i, flags;
		struct freenect_device_attributes* attribute_list;
		struct freenect_device_attributes* attribute;
	
		// list devices:
		if (!f_ctx) return;
		
		int num_devices = freenect_list_device_attributes(f_ctx, &attribute_list);
		
		i = 0;
		attribute = attribute_list;
		while (attribute) {
			atom_setsym(a+i, gensym(attribute->camera_serial));
			attribute = attribute->next;
		}
		outlet_anything(outlet_msg, gensym("devlist"), num_devices, a);
		
		freenect_free_device_attributes(attribute_list);
		
		//post("supported flags: %d", freenect_supported_subdevices());
	}
	
	
	void open(t_symbol *s, long argc, t_atom *argv) {
	
		if (device){
			object_post(&ob, "A device is already open.");
			return;
		}
		
		// mark one additional device:
		capturing++;
		
		if(!f_ctx){
			long priority = 0; // maybe increase?
			if (systhread_create((method)&capture_threadfunc, this, 0, priority, 0, &capture_thread)) {
				object_error(&ob, "Failed to create capture thread.");
				capturing = 0;
				return;
			}
			while(!f_ctx){
				systhread_sleep(0);
			}
		}
		
		int ndevices = freenect_num_devices(f_ctx);
		if(!ndevices){
			object_post(&ob, "Could not find any connected Kinect device. Are you sure the power cord is plugged-in?");
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
			capturing--;
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
		freenect_start_video(device);
	}
	
	void close() {
		if(!device) return;
		
		freenect_set_led(device,LED_BLINK_GREEN);
		freenect_close_device(device);
		device = NULL;
		
		// mark one less device running:
		capturing--;
	}
	
	void accel() {
		t_atom a[3];
		double x, y, z;
		
		
		if (!device) return;
		
		freenect_update_tilt_state(device);
		freenect_raw_tilt_state * state = freenect_get_tilt_state(device);
		
		freenect_get_mks_accel(state, &x, &y, &z);
		atom_setfloat(a+0, x);
		atom_setfloat(a+1, y);
		atom_setfloat(a+2, z);
		outlet_anything(outlet_msg, gensym("accel"), 3, a);
	}
	
	void led(int option) {
		if (!device) return;
		
//		LED_OFF              = 0, /**< Turn LED off */
//		LED_GREEN            = 1, /**< Turn LED to Green */
//		LED_RED              = 2, /**< Turn LED to Red */
//		LED_YELLOW           = 3, /**< Turn LED to Yellow */
//		LED_BLINK_GREEN      = 4, /**< Make LED blink Green */
//		// 5 is same as 4, LED blink Green
//		LED_BLINK_RED_YELLOW = 6, /**< Make LED blink Red/Yellow */
		freenect_set_led(device, (freenect_led_options)option);
	}
	
	void depth_process() {
		// for each cell:
		for (int i=0; i<DEPTH_HEIGHT*DEPTH_WIDTH; i++) {
			// cache raw, unrectified depth in output:
			// (casts uint16_t to uint32_t)
			depth_back[i] = depth_data[i];
		}
		new_depth_data = 1;

		cloud_process();
	}
	
	static void rgb_callback(freenect_device *dev, void *pixels, uint32_t timestamp){
		t_kinect *x = (t_kinect *)freenect_get_user(dev);
		if(!x)return;
		
		x->cloud_rgb_process();
		
		x->new_rgb_data = 1;
	}
	
	static void depth_callback(freenect_device *dev, void *pixels, uint32_t timestamp){
		t_kinect *x = (t_kinect *)freenect_get_user(dev);
		if(!x)return;
		
		// consider moving this to another thread to avoid dropouts?
		x->depth_process();
	}
	
	static void log_cb(freenect_context *dev, freenect_loglevel level, const char *msg) {
		post(msg);
	}
	
	static void *capture_threadfunc(void *arg) {
		t_kinect *x = (t_kinect *)arg;
		
		// create the freenect context:
		freenect_context *context;		
		if(!f_ctx){
			if (freenect_init(&context, NULL) < 0) {
				error("freenect_init() failed");
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
		post("freenect starting processing");
		while (capturing > 0) {
			int err = freenect_process_events(f_ctx);
			//int err = freenect_process_events_timeout(f_ctx);
			if(err < 0){
				error("Freenect could not process events.");
				break;
			}
			systhread_sleep(0);
		}
		post("freenect finished processing");
		
	out: 
		freenect_shutdown(f_ctx);
		f_ctx = NULL;
		systhread_exit(NULL);
		return NULL;
	}
};