#ifndef max_kinect_h
#define max_kinect_h

extern "C" {
	#include "ext.h"
	#include "ext_obex.h"
	#include "ext_dictionary.h"
	#include "ext_dictobj.h"
	#include "ext_systhread.h"
		
	#include "jit.common.h"
	#include "jit.gl.h"
}

#ifdef __GNUC__
#include <stdint.h>
#else
#include "msc_stdint.h"
#endif

// how many glm headers do we really need?
#define GLM_FORCE_RADIANS
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/noise.hpp"
#include "glm/gtc/random.hpp"
#include "glm/gtc/type_ptr.hpp"

// unstable extensions
#include "glm/gtx/norm.hpp"

//typedef glm::vec2 vec2f;
//typedef glm::vec3 vec3f;
//typedef glm::vec4 vec4f;
//typedef glm::quat quatf;
//
//typedef glm::dvec2 vec2;
//typedef glm::dvec3 vec3;
//typedef glm::dvec4 vec4;
//typedef glm::dquat quat;
//
//typedef glm::fmat3 mat3f;
//typedef glm::fmat4 mat4f;
//typedef glm::dmat3 mat3;
//typedef glm::dmat4 mat4;
//
//typedef glm::i8vec2 vec2c;
//typedef glm::i8vec3 vec3c;
//typedef glm::i8vec4 vec4c;
//
//typedef glm::i32vec2 vec2i;
//typedef glm::i32vec3 vec3i;
//typedef glm::i32vec4 vec4i;

#define KINECT_DEPTH_WIDTH 640
#define KINECT_DEPTH_HEIGHT 480

struct KinectBase {
	t_object	ob;			// the object itself (must be first)
	
	// new data flags for unique mode:
	volatile char new_rgb_data;
	volatile char new_depth_data;
	volatile char new_cloud_data;
	
	// rgb matrix for raw output:
	void *		rgb_mat;
	void *		rgb_mat_wrapper;
	t_atom		rgb_name[1];
	glm::i8vec3 * rgb_back;
	
	// depth matrix for raw output:
	void *		depth_mat;
	void *		depth_mat_wrapper;
	t_atom		depth_name[1];
	uint32_t *	depth_back;
	
	// cloud matrix for output:
	void *		cloud_mat;
	void *		cloud_mat_wrapper;
	t_atom		cloud_name[1];
	glm::vec3 *	cloud_back;
	
	// pose of cloud:
	glm::vec3	cloud_position;
	glm::quat	cloud_quat;
	
	// attrs:
	int			unique;	// whether we output whenever there is a bang, or only when there is new data
	int			use_rgb; // whether to output RGB, or depth only
	int			align_rgb_to_cloud;	// output RGB image warped to fit cloud
	
	// outlets:
	void *		outlet_cloud;
	void *		outlet_rgb;
	void *		outlet_depth;
	void *		outlet_player;
	void *		outlet_skeleton;
	void *		outlet_msg;
	
	KinectBase() {
		
		unique = 1;
		new_rgb_data = 0;
		new_depth_data = 0;
		new_cloud_data = 0;
		
		use_rgb = 1;
		align_rgb_to_cloud = 0;
		
		outlet_msg = outlet_new(&ob, 0);
		outlet_skeleton = outlet_new(&ob, 0);
		outlet_player = outlet_new(&ob, "jit_matrix");
		outlet_rgb = outlet_new(&ob, "jit_matrix");
		outlet_depth = outlet_new(&ob, "jit_matrix");
		outlet_cloud = outlet_new(&ob, "jit_matrix");
		
		// initialize in jitter format:
		cloud_quat.x = 0.;
		cloud_quat.y = 0.;
		cloud_quat.z = 0.;
		cloud_quat.w = 1.;
		
		// create matrices:
		{
			t_jit_matrix_info info;
			rgb_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
			rgb_mat = jit_object_method(rgb_mat_wrapper, _jit_sym_getmatrix);
			// create the internal data:
			jit_matrix_info_default(&info);
			info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
			info.planecount = 3;
			info.type = gensym("char");
			info.dimcount = 2;
			info.dim[0] = KINECT_DEPTH_WIDTH;
			info.dim[1] = KINECT_DEPTH_HEIGHT;
			jit_object_method(rgb_mat, _jit_sym_setinfo_ex, &info);
			jit_object_method(rgb_mat, _jit_sym_clear);
			jit_object_method(rgb_mat, _jit_sym_getdata, &rgb_back);
			// cache name:
			atom_setsym(rgb_name, jit_attr_getsym(rgb_mat_wrapper, _jit_sym_name));
		}
		
		{
			t_jit_matrix_info info;
			depth_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
			depth_mat = jit_object_method(depth_mat_wrapper, _jit_sym_getmatrix);
			// create the internal data:
			jit_matrix_info_default(&info);
			info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
			info.planecount = 1;
			info.type = gensym("long");
			info.dimcount = 2;
			info.dim[0] = KINECT_DEPTH_WIDTH;
			info.dim[1] = KINECT_DEPTH_HEIGHT;
			jit_object_method(depth_mat, _jit_sym_setinfo_ex, &info);
			jit_object_method(depth_mat, _jit_sym_clear);
			jit_object_method(depth_mat, _jit_sym_getdata, &depth_back);
			// cache name:
			atom_setsym(depth_name, jit_attr_getsym(depth_mat_wrapper, _jit_sym_name));
		}
		
		{
			t_jit_matrix_info info;
			cloud_mat_wrapper = jit_object_new(gensym("jit_matrix_wrapper"), jit_symbol_unique(), 0, NULL);
			cloud_mat = jit_object_method(cloud_mat_wrapper, _jit_sym_getmatrix);
			// create the internal data:
			jit_matrix_info_default(&info);
			info.flags |= JIT_MATRIX_DATA_PACK_TIGHT;
			info.planecount = 3;
			info.type = gensym("float32");
			info.dimcount = 2;
			info.dim[0] = KINECT_DEPTH_WIDTH;
			info.dim[1] = KINECT_DEPTH_HEIGHT;
			jit_object_method(cloud_mat, _jit_sym_setinfo_ex, &info);
			jit_object_method(cloud_mat, _jit_sym_clear);
			jit_object_method(cloud_mat, _jit_sym_getdata, &cloud_back);
			// cache name:
			atom_setsym(cloud_name, jit_attr_getsym(cloud_mat_wrapper, _jit_sym_name));
		}
	}
	
	~KinectBase() {
		if (rgb_mat_wrapper) {
			object_free(rgb_mat_wrapper);
			rgb_mat_wrapper = NULL;
		}
		if (depth_mat_wrapper) {
			object_free(depth_mat_wrapper);
			depth_mat_wrapper = NULL;
		}
//		if (player_mat_wrapper) {
//			object_free(player_mat_wrapper);
//			player_mat_wrapper = NULL;
//		}
		if (cloud_mat_wrapper) {
			object_free(cloud_mat_wrapper);
			cloud_mat_wrapper = NULL;
		}
	}
	
	void bang() {
		if (unique) {
			if (align_rgb_to_cloud) {
				if (new_depth_data) {
					outlet_anything(outlet_depth, _jit_sym_jit_matrix, 1, depth_name);
					new_depth_data = 0;
				}
				
				if (new_cloud_data) {
					outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_name  );
					outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, cloud_name);
					new_cloud_data = 0;
				}
			} else {
				if (new_depth_data) {
					outlet_anything(outlet_depth, _jit_sym_jit_matrix, 1, depth_name);
				}
				new_depth_data = 0;
				
				if (new_rgb_data) {
					outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_name  );
				}
				new_rgb_data = 0;
				
				if (new_cloud_data) {
					outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, cloud_name);
				}
				new_cloud_data = 0;
			}
		} else {
			outlet_anything(outlet_depth, _jit_sym_jit_matrix, 1, depth_name);
			outlet_anything(outlet_rgb  , _jit_sym_jit_matrix, 1, rgb_name  );
			outlet_anything(outlet_cloud, _jit_sym_jit_matrix, 1, cloud_name);
		}
	}
};

#endif
