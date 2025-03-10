# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rb/api/parameter.proto
# Protobuf Python Version: 5.26.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from rb.api import header_pb2 as rb_dot_api_dot_header__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x16rb/api/parameter.proto\x12\x06rb.api\x1a\x13rb/api/header.proto\"R\n\x13GetParameterRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\x12\x0c\n\x04name\x18\x02 \x01(\t\"Z\n\x14GetParameterResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\x12\x11\n\tparameter\x18\x02 \x01(\t\"e\n\x13SetParameterRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\x12\x0c\n\x04name\x18\x02 \x01(\t\x12\x11\n\tparameter\x18\x03 \x01(\t\"G\n\x14SetParameterResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\"H\n\x17GetParameterListRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\"\xbc\x01\n\x18GetParameterListResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\x12\x42\n\nparameters\x18\x02 \x03(\x0b\x32..rb.api.GetParameterListResponse.ParameterType\x1a+\n\rParameterType\x12\x0c\n\x04name\x18\x01 \x01(\t\x12\x0c\n\x04type\x18\x02 \x01(\x05\"[\n\x1c\x46\x61\x63toryResetParameterRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\x12\x0c\n\x04name\x18\x02 \x01(\t\"P\n\x1d\x46\x61\x63toryResetParameterResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\"Q\n FactoryResetAllParametersRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\"T\n!FactoryResetAllParametersResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\"T\n\x15ResetParameterRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\x12\x0c\n\x04name\x18\x02 \x01(\t\"I\n\x16ResetParameterResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\"J\n\x19ResetAllParametersRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\"M\n\x1aResetAllParametersResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\"]\n\x1eResetParameterToDefaultRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\x12\x0c\n\x04name\x18\x02 \x01(\t\"R\n\x1fResetParameterToDefaultResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeader\"S\n\"ResetAllParametersToDefaultRequest\x12-\n\x0erequest_header\x18\x01 \x01(\x0b\x32\x15.rb.api.RequestHeader\"V\n#ResetAllParametersToDefaultResponse\x12/\n\x0fresponse_header\x18\x01 \x01(\x0b\x32\x16.rb.api.ResponseHeaderb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'rb.api.parameter_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_GETPARAMETERREQUEST']._serialized_start=55
  _globals['_GETPARAMETERREQUEST']._serialized_end=137
  _globals['_GETPARAMETERRESPONSE']._serialized_start=139
  _globals['_GETPARAMETERRESPONSE']._serialized_end=229
  _globals['_SETPARAMETERREQUEST']._serialized_start=231
  _globals['_SETPARAMETERREQUEST']._serialized_end=332
  _globals['_SETPARAMETERRESPONSE']._serialized_start=334
  _globals['_SETPARAMETERRESPONSE']._serialized_end=405
  _globals['_GETPARAMETERLISTREQUEST']._serialized_start=407
  _globals['_GETPARAMETERLISTREQUEST']._serialized_end=479
  _globals['_GETPARAMETERLISTRESPONSE']._serialized_start=482
  _globals['_GETPARAMETERLISTRESPONSE']._serialized_end=670
  _globals['_GETPARAMETERLISTRESPONSE_PARAMETERTYPE']._serialized_start=627
  _globals['_GETPARAMETERLISTRESPONSE_PARAMETERTYPE']._serialized_end=670
  _globals['_FACTORYRESETPARAMETERREQUEST']._serialized_start=672
  _globals['_FACTORYRESETPARAMETERREQUEST']._serialized_end=763
  _globals['_FACTORYRESETPARAMETERRESPONSE']._serialized_start=765
  _globals['_FACTORYRESETPARAMETERRESPONSE']._serialized_end=845
  _globals['_FACTORYRESETALLPARAMETERSREQUEST']._serialized_start=847
  _globals['_FACTORYRESETALLPARAMETERSREQUEST']._serialized_end=928
  _globals['_FACTORYRESETALLPARAMETERSRESPONSE']._serialized_start=930
  _globals['_FACTORYRESETALLPARAMETERSRESPONSE']._serialized_end=1014
  _globals['_RESETPARAMETERREQUEST']._serialized_start=1016
  _globals['_RESETPARAMETERREQUEST']._serialized_end=1100
  _globals['_RESETPARAMETERRESPONSE']._serialized_start=1102
  _globals['_RESETPARAMETERRESPONSE']._serialized_end=1175
  _globals['_RESETALLPARAMETERSREQUEST']._serialized_start=1177
  _globals['_RESETALLPARAMETERSREQUEST']._serialized_end=1251
  _globals['_RESETALLPARAMETERSRESPONSE']._serialized_start=1253
  _globals['_RESETALLPARAMETERSRESPONSE']._serialized_end=1330
  _globals['_RESETPARAMETERTODEFAULTREQUEST']._serialized_start=1332
  _globals['_RESETPARAMETERTODEFAULTREQUEST']._serialized_end=1425
  _globals['_RESETPARAMETERTODEFAULTRESPONSE']._serialized_start=1427
  _globals['_RESETPARAMETERTODEFAULTRESPONSE']._serialized_end=1509
  _globals['_RESETALLPARAMETERSTODEFAULTREQUEST']._serialized_start=1511
  _globals['_RESETALLPARAMETERSTODEFAULTREQUEST']._serialized_end=1594
  _globals['_RESETALLPARAMETERSTODEFAULTRESPONSE']._serialized_start=1596
  _globals['_RESETALLPARAMETERSTODEFAULTRESPONSE']._serialized_end=1682
# @@protoc_insertion_point(module_scope)
