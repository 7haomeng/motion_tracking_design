## *********************************************************
##
## File autogenerated for the yolact_ros package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 246, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 291, 'description': 'Image topic used for subscribing', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'image_topic', 'edit_method': '', 'default': '/camera/color/image_raw', 'level': 1, 'min': '', 'type': 'str'}, {'srcline': 291, 'description': 'Subscribe to compressed image topic', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'use_compressed_image', 'edit_method': '', 'default': False, 'level': 1, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Publish images with detections', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'publish_visualization', 'edit_method': '', 'default': True, 'level': 2, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Publish detections as message', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'publish_detections', 'edit_method': '', 'default': True, 'level': 4, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Display window with detection image', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'display_visualization', 'edit_method': '', 'default': False, 'level': 8, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Whether or not to display masks over bounding boxes', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'display_masks', 'edit_method': '', 'default': True, 'level': 16, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Whether or not to display bboxes around masks', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'display_bboxes', 'edit_method': '', 'default': True, 'level': 32, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Whether or not to display text (class [score])', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'display_text', 'edit_method': '', 'default': True, 'level': 64, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Whether or not to display scores in addition to classes', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'display_scores', 'edit_method': '', 'default': True, 'level': 128, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'When displaying video, draw the FPS on the frame', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'display_fps', 'edit_method': '', 'default': False, 'level': 256, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Detections with a score under this threshold will not be considered', 'max': 1.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'score_threshold', 'edit_method': '', 'default': 0.0, 'level': 512, 'min': 0.0, 'type': 'double'}, {'srcline': 291, 'description': 'If true, crop output masks with the predicted bounding box', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'crop_masks', 'edit_method': '', 'default': True, 'level': 1024, 'min': False, 'type': 'bool'}, {'srcline': 291, 'description': 'Further restrict the number of predictions to parse', 'max': 100, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/melodic/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'name': 'top_k', 'edit_method': '', 'default': 5, 'level': 2048, 'min': 0, 'type': 'int'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']
