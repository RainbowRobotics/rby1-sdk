# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rb/api/geometry.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x15rb/api/geometry.proto\x12\x06rb.api\"\x1c\n\x04Vec2\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\"\'\n\x04Vec3\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\"8\n\x07SE2Pose\x12\x1e\n\x08position\x18\x01 \x01(\x0b\x32\x0c.rb.api.Vec2\x12\r\n\x05\x61ngle\x18\x02 \x01(\x01\"<\n\x0bSE2Velocity\x12\x1c\n\x06linear\x18\x01 \x01(\x0b\x32\x0c.rb.api.Vec2\x12\x0f\n\x07\x61ngular\x18\x02 \x01(\x01\"8\n\nQuaternion\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\x12\t\n\x01w\x18\x04 \x01(\x01\"0\n\rEulerAngleZYX\x12\t\n\x01z\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01x\x18\x03 \x01(\x01\"\x87\x01\n\x07SE3Pose\x12\x1e\n\x08position\x18\x01 \x01(\x0b\x32\x0c.rb.api.Vec3\x12(\n\nquaternion\x18\x02 \x01(\x0b\x32\x12.rb.api.QuaternionH\x00\x12&\n\x05\x65uler\x18\x03 \x01(\x0b\x32\x15.rb.api.EulerAngleZYXH\x00\x42\n\n\x08rotation\"`\n\x08Inertial\x12\x0c\n\x04mass\x18\x01 \x01(\x01\x12$\n\x0e\x63\x65nter_of_mass\x18\x02 \x01(\x0b\x32\x0c.rb.api.Vec3\x12 \n\x07inertia\x18\x03 \x01(\x0b\x32\x0f.rb.api.Inertia\"W\n\x07Inertia\x12\x0b\n\x03ixx\x18\x01 \x01(\x01\x12\x0b\n\x03iyy\x18\x02 \x01(\x01\x12\x0b\n\x03izz\x18\x03 \x01(\x01\x12\x0b\n\x03ixy\x18\x04 \x01(\x01\x12\x0b\n\x03ixz\x18\x05 \x01(\x01\x12\x0b\n\x03iyz\x18\x06 \x01(\x01\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'rb.api.geometry_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_VEC2']._serialized_start=33
  _globals['_VEC2']._serialized_end=61
  _globals['_VEC3']._serialized_start=63
  _globals['_VEC3']._serialized_end=102
  _globals['_SE2POSE']._serialized_start=104
  _globals['_SE2POSE']._serialized_end=160
  _globals['_SE2VELOCITY']._serialized_start=162
  _globals['_SE2VELOCITY']._serialized_end=222
  _globals['_QUATERNION']._serialized_start=224
  _globals['_QUATERNION']._serialized_end=280
  _globals['_EULERANGLEZYX']._serialized_start=282
  _globals['_EULERANGLEZYX']._serialized_end=330
  _globals['_SE3POSE']._serialized_start=333
  _globals['_SE3POSE']._serialized_end=468
  _globals['_INERTIAL']._serialized_start=470
  _globals['_INERTIAL']._serialized_end=566
  _globals['_INERTIA']._serialized_start=568
  _globals['_INERTIA']._serialized_end=655
# @@protoc_insertion_point(module_scope)
