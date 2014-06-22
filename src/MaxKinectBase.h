/**
	@file
	kinect - a max object
	
	
	Q: is it better to have a separate freenect_context per device?
	
*/

extern "C" {
	#include "ext.h"		
	#include "ext_obex.h"
	#include "ext_dictionary.h"
	#include "ext_dictobj.h"
	#include "ext_systhread.h"

	#include "jit.common.h"
	#include "jit.gl.h"
}

#include <new>
#include "stdint.h"

#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480

class MaxKinectBase {
public:

	struct vec2i { int x, y; };
	struct vec2f { float x, y; };
	struct vec3f { float x, y, z; };

	t_object	ob;			// the object itself (must be first)

	void *		outlet_cloud;
	void *		outlet_rgb;
	void *		outlet_depth;
	void *		outlet_msg;
	
	// rgb matrix for raw output:
	void *		rgb_mat;
	void *		rgb_mat_wrapper;
	t_atom		rgb_name[1];
	uint8_t *	rgb_back;
	
	// depth matrix for raw output:
	void *		depth_mat;
	void *		depth_mat_wrapper;
	t_atom		depth_name[1];
	uint32_t *	depth_back;
	
	// cloud matrix for output:
	void *		cloud_mat;
	void *		cloud_mat_wrapper;
	t_atom		cloud_name[1];
	vec3f *		cloud_back;
	
	// attributes:
	vec2f		depth_focal;
	vec2f		depth_center;
	float		depth_base, depth_offset;
	int			unique;
	int			device_count;
	int			near_mode;
	int			player;
	
	vec2i *		depth_map_data;
	
	volatile char new_rgb_data;
	volatile char new_depth_data;
	volatile char new_cloud_data;
	
	MaxKinectBase() {
		// set up attrs:
		unique = 1;
		near_mode = 0;
		device_count = 0;
		player = 0;
		
		new_rgb_data = 0;
		new_depth_data = 0;
		new_cloud_data = 0;
		
		// can we accept a dict?
		depth_base = 0.085f;
		depth_offset = 0.0011f;
		depth_center.x = 314.f;
		depth_center.y = 241.f;
		depth_focal.x = 597.f;
		depth_focal.y = 597.f;
		
		// create matrices:
		t_jit_matrix_info info;
		
		rgb_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		rgb_mat = jit_object_method(rgb_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 3;
		info.type = gensym("char");
		info.dimcount = 2;
		info.dim[0] = DEPTH_WIDTH;
		info.dim[1] = DEPTH_HEIGHT;
		jit_object_method(rgb_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(rgb_mat, _jit_sym_clear);
		jit_object_method(rgb_mat, _jit_sym_getdata, &rgb_back);
		// cache name:
		atom_setsym(rgb_name, jit_attr_getsym(rgb_mat_wrapper, _jit_sym_name));
		
		depth_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		depth_mat = jit_object_method(depth_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 1;
		info.type = gensym("long");
		info.dimcount = 2;
		info.dim[0] = DEPTH_WIDTH;
		info.dim[1] = DEPTH_HEIGHT;
		jit_object_method(depth_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(depth_mat, _jit_sym_clear);
		jit_object_method(depth_mat, _jit_sym_getdata, &depth_back);
		// cache name:
		atom_setsym(depth_name, jit_attr_getsym(depth_mat_wrapper, _jit_sym_name));
		
		cloud_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
		cloud_mat = jit_object_method(cloud_mat_wrapper, _jit_sym_getmatrix);
		// create the internal data:
		jit_matrix_info_default(&info);
		info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
		info.planecount = 3;
		info.type = gensym("float32");
		info.dimcount = 1;
		info.dim[0] = DEPTH_WIDTH * DEPTH_HEIGHT;
		jit_object_method(cloud_mat, _jit_sym_setinfo_ex, &info);
		jit_object_method(cloud_mat, _jit_sym_clear);
		jit_object_method(cloud_mat, _jit_sym_getdata, &cloud_back);
		// cache name:
		atom_setsym(cloud_name, jit_attr_getsym(cloud_mat_wrapper, _jit_sym_name));
		
		// init depth_map with default data:
		depth_map_data = (vec2i *)sysmem_newptr(DEPTH_WIDTH*DEPTH_HEIGHT * sizeof(vec2i));
		for (int i=0, y=0; y<DEPTH_HEIGHT; y++) {
			for (int x=0; x<DEPTH_WIDTH; x++, i++) {
				depth_map_data[i].x = x;
				depth_map_data[i].y = y;
			}
		}
	}
	
	~MaxKinectBase() {
		if (rgb_mat_wrapper) {
			object_free(rgb_mat_wrapper);
			rgb_mat_wrapper = NULL;
		}
		if (depth_mat_wrapper) {
			object_free(depth_mat_wrapper);
			depth_mat_wrapper = NULL;
		}
		if (cloud_mat_wrapper) {
			object_free(cloud_mat_wrapper);
			cloud_mat_wrapper = NULL;
		}
		sysmem_freeptr(depth_map_data);
	}
	
	void depth_map(t_symbol * name) {
		t_jit_matrix_info in_info;
		long in_savelock;
		char * in_bp;
		t_jit_err err = 0;
		
		// get matrix from name:
		void * in_mat = jit_object_findregistered(name);
		if (!in_mat) {
			object_error(&ob, "failed to acquire matrix");
			err = JIT_ERR_INVALID_INPUT;
			goto out;
		}
		
		// lock it:
		in_savelock = (long)jit_object_method(in_mat, _jit_sym_lock, 1);
		
		// first ensure the type is correct:
		jit_object_method(in_mat, _jit_sym_getinfo, &in_info);
		jit_object_method(in_mat, _jit_sym_getdata, &in_bp);
		if (!in_bp) {
			err = JIT_ERR_INVALID_INPUT;
			goto unlock;
		}
		
		if (in_info.planecount != 2) {
			err = JIT_ERR_MISMATCH_PLANE;
			goto unlock;
		}
		
		if (in_info.type != _jit_sym_float32) {
			err = JIT_ERR_MISMATCH_TYPE;
			goto unlock;
		}
		
		if (in_info.dimcount != 2 || in_info.dim[0] != DEPTH_WIDTH || in_info.dim[1] != DEPTH_HEIGHT) {
			err = JIT_ERR_MISMATCH_DIM;
			goto unlock;
		}

		// copy matrix data into depth map:
		for (int i=0, y=0; y<DEPTH_HEIGHT; y++) {
			// get row pointer:
			char * ip = in_bp + y*in_info.dimstride[1];
			
			for (int x=0; x<DEPTH_WIDTH; x++, i++) {
				
				// convert column pointer to vec2f:
				const vec2f& v = *(vec2f *)(ip);
				
				//post("%i %i: %i %f %f", x, y, i, v.x, v.y);
			
				// +0.5 for nice rounding -- TODO is this necessary?
				int ix = (int)(v.x + 0.5);
				int iy = (int)(v.y + 0.5);
				
				// clip at boundaries:
				ix = ix < 0 ? 0 : ix >= DEPTH_WIDTH ? DEPTH_WIDTH-1 : ix;
				iy = iy < 0 ? 0 : iy >= DEPTH_HEIGHT ? DEPTH_HEIGHT-1 : iy;
				
				// store:
				depth_map_data[i].x = ix;
				depth_map_data[i].y = iy;
				
				// move to next column:
				ip += in_info.dimstride[0];
			}
		}
		
	unlock:
		// restore matrix lock state:
		jit_object_method(in_mat, _jit_sym_lock, in_savelock);
	out:
		if (err) {
			jit_error_code(&ob, err);
		}
	}
	
	void bang() {
		if (unique) {
			if (new_rgb_data) {
				new_rgb_data = 0;
				outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_name  );
			}
			if (new_depth_data) {
				new_depth_data = 0;
				outlet_anything(outlet_depth, _jit_sym_jit_matrix, 1, depth_name);
			}
			if (new_cloud_data) {
				new_cloud_data = 0;
				outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, cloud_name);
			}
		} else {
			outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_name  );
			outlet_anything(outlet_depth, _jit_sym_jit_matrix, 1, depth_name);
			outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, cloud_name);
		}

	}
	
	void cloud_process() {
		float inv_depth_focal_x = 1.f/depth_focal.x;
		float inv_depth_focal_y = 1.f/depth_focal.y;
	
		// for each cell:
		for (int i=0, y=0; y<DEPTH_HEIGHT; y++) {
			for (int x=0; x<DEPTH_WIDTH; x++, i++) {
				
				// remove the effects of lens distortion
				// (lookup into distortion map)
				vec2i di = depth_map_data[i];
				uint16_t d = depth_back[di.x + di.y*DEPTH_WIDTH];
				
				// Of course this isn't optimal; actually we should be doing the reverse, 
				// i.e. converting cell index to match the distortion. 
				// But it's not trivial to invert the lens distortion.
				
				//if (d < 2047) {
					// convert pixel coordinate to NDC depth plane intersection
					float uv_x = (x - depth_center.x) * inv_depth_focal_x;
					float uv_y = (y - depth_center.y) * inv_depth_focal_y;
					
					// convert Kinect depth to Z
					// NOTE: this should be cached into a lookup table
					// TODO: what are these magic numbers? the result is meters					
					
					// convert raw disparity to meters
					float z = d * 0.001f;

					// and scale according to depth (projection)
					uv_x = uv_x * z;
					uv_y = uv_y * z;
					
					// flip for GL:
					cloud_back[i].x = uv_x;
					cloud_back[i].y = -uv_y;
					cloud_back[i].z = -z;
					
				//} else {
//				
//					cloud_back[i].x = 0;
//					cloud_back[i].y = 0;
//					cloud_back[i].z = 0;
//				}
			}
		}
		new_cloud_data = 1;
	}
	
	void dictionary(t_symbol *s, long argc, t_atom *argv) {
		
		t_dictionary	*d = dictobj_findregistered_retain(s);
		long			numkeys = 0;
		t_symbol		**keys = NULL;
		int				i;
//		int				size;

		if (!d) {
			object_error(&ob, "unable to reference dictionary named %s", s);
			return;
		}


		dictionary_getkeys_ordered(d, &numkeys, &keys);
		for (i=0; i<numkeys; i++) {
			long	argc = 0;
			t_atom	*argv = NULL;
			t_symbol * key = keys[i];
			
			post("key %s", keys[i]->s_name);
			
			/*
			key R
			key T
			key depth_base_and_offset
			key depth_distortion
			key depth_intrinsics
			key depth_size
			key raw_depth_size
			key raw_rgb_size
			key rgb_distortion
			key rgb_intrinsics
			key rgb_size
			*/
			
			if (key == gensym("depth_base_and_offset")) {
				
			}

			/*
			
			dictionary_getatoms(d, keys[i], &argc, &argv);
			if (argc == 1 && atom_gettype(argv) == A_OBJ && object_classname(atom_getobj(argv)) == _sym_dictionary) {
				t_atom			a[2];
				t_dictionary	*d2 = (t_dictionary *) atom_getobj(argv);
				t_symbol		*d2_name = dictobj_namefromptr(d2);

				// if we have a dictionary, but it doesn't have a name,
				// then we register it so that it can be passed out and accessed by other dict objects
				// we don't want to leak this memory, however, so we track it with a linklist and clear it when we are done
				//
				// this means that operations on the output from dict.iter must be synchronous
				// the dict world generally presupposes synchronous operation, so maybe it's not that bad?

				if (!subdictionaries) {
					subdictionaries = linklist_new();
					linklist_flags(subdictionaries, OBJ_FLAG_DATA);
				}

				if (!d2_name) {
					d2 = dictobj_register(d2, &d2_name);
					linklist_append(subdictionaries, d2);
				}

				atom_setsym(a, _sym_dictionary);
				atom_setsym(a+1, d2_name);
				outlet_anything(x->outlet, keys[i], 2, a);
			}
			else
				outlet_anything(x->outlet, keys[i], argc, argv);
			*/
		}

		dictobj_release(d);
		if (keys)
			sysmem_freeptr(keys);
	}
};