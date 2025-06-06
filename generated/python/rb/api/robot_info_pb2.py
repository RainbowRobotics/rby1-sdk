# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rb/api/robot_info.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from rb.api import header_pb2 as rb_dot_api_dot_header__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x17rb/api/robot_info.proto\x12\x06rb.api\x1a\x13rb/api/header.proto\"\r\n\x0b\x42\x61tteryInfo\"\x19\n\tPowerInfo\x12\x0c\n\x04name\x18\x01 \x01(\t\"\x17\n\x07\x45MOInfo\x12\x0c\n\x04name\x18\x01 \x01(\t\"\\\n\tJointInfo\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x11\n\thas_brake\x18\x02 \x01(\x08\x12\x14\n\x0cproduct_name\x18\x03 \x01(\t\x12\x18\n\x10\x66irmware_version\x18\x04 \x01(\t\"\xab\x03\n\tRobotInfo\x12\x15\n\rrobot_version\x18\x01 \x01(\t\x12\x18\n\x10robot_model_name\x18\x0c \x01(\t\x12\x15\n\rsdk_commit_id\x18\n \x01(\t\x12)\n\x0c\x62\x61ttery_info\x18\x02 \x01(\x0b\x32\x13.rb.api.BatteryInfo\x12&\n\x0bpower_infos\x18\x03 \x03(\x0b\x32\x11.rb.api.PowerInfo\x12\"\n\temo_infos\x18\x0b \x03(\x0b\x32\x0f.rb.api.EMOInfo\x12\x19\n\x11\x64\x65gree_of_freedom\x18\x04 \x01(\x05\x12&\n\x0bjoint_infos\x18\x05 \x03(\x0b\x32\x11.rb.api.JointInfo\x12\x1a\n\x12mobility_joint_idx\x18\x06 \x03(\r\x12\x16\n\x0e\x62ody_joint_idx\x18\x07 \x03(\r\x12\x16\n\x0ehead_joint_idx\x18\x08 \x03(\r\x12\x17\n\x0ftorso_joint_idx\x18\r \x03(\r\x12\x1b\n\x13right_arm_joint_idx\x18\x0e \x03(\r\x12\x1a\n\x12left_arm_joint_idx\x18\x0f \x03(\r\"D\n\x13GetRobotInfoRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\"n\n\x14GetRobotInfoResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\x12%\n\nrobot_info\x18\x02 \x01(\x0b\x32\x11.rb.api.RobotInfo\"E\n\x14GetRobotModelRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\"W\n\x15GetRobotModelResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\x12\r\n\x05model\x18\x02 \x01(\t\"e\n\x17ImportRobotModelRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\x12\x0c\n\x04name\x18\x02 \x01(\t\x12\r\n\x05model\x18\x03 \x01(\t\"K\n\x18ImportRobotModelResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeaderb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'rb.api.robot_info_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_BATTERYINFO']._serialized_start=56
  _globals['_BATTERYINFO']._serialized_end=69
  _globals['_POWERINFO']._serialized_start=71
  _globals['_POWERINFO']._serialized_end=96
  _globals['_EMOINFO']._serialized_start=98
  _globals['_EMOINFO']._serialized_end=121
  _globals['_JOINTINFO']._serialized_start=123
  _globals['_JOINTINFO']._serialized_end=215
  _globals['_ROBOTINFO']._serialized_start=218
  _globals['_ROBOTINFO']._serialized_end=645
  _globals['_GETROBOTINFOREQUEST']._serialized_start=647
  _globals['_GETROBOTINFOREQUEST']._serialized_end=715
  _globals['_GETROBOTINFORESPONSE']._serialized_start=717
  _globals['_GETROBOTINFORESPONSE']._serialized_end=827
  _globals['_GETROBOTMODELREQUEST']._serialized_start=829
  _globals['_GETROBOTMODELREQUEST']._serialized_end=898
  _globals['_GETROBOTMODELRESPONSE']._serialized_start=900
  _globals['_GETROBOTMODELRESPONSE']._serialized_end=987
  _globals['_IMPORTROBOTMODELREQUEST']._serialized_start=989
  _globals['_IMPORTROBOTMODELREQUEST']._serialized_end=1090
  _globals['_IMPORTROBOTMODELRESPONSE']._serialized_start=1092
  _globals['_IMPORTROBOTMODELRESPONSE']._serialized_end=1167
# @@protoc_insertion_point(module_scope)
