#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__srv__MusicPlay_Request() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__srv__MusicPlay_Request__init(msg: *mut MusicPlay_Request) -> bool;
    fn audio_common_msgs__srv__MusicPlay_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MusicPlay_Request>, size: usize) -> bool;
    fn audio_common_msgs__srv__MusicPlay_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MusicPlay_Request>);
    fn audio_common_msgs__srv__MusicPlay_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MusicPlay_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<MusicPlay_Request>) -> bool;
}

// Corresponds to audio_common_msgs__srv__MusicPlay_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MusicPlay_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub audio: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub file_path: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub loop_: bool,

}



impl Default for MusicPlay_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__srv__MusicPlay_Request__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__srv__MusicPlay_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MusicPlay_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__srv__MusicPlay_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__srv__MusicPlay_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__srv__MusicPlay_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MusicPlay_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MusicPlay_Request where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/srv/MusicPlay_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__srv__MusicPlay_Request() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__srv__MusicPlay_Response() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__srv__MusicPlay_Response__init(msg: *mut MusicPlay_Response) -> bool;
    fn audio_common_msgs__srv__MusicPlay_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MusicPlay_Response>, size: usize) -> bool;
    fn audio_common_msgs__srv__MusicPlay_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MusicPlay_Response>);
    fn audio_common_msgs__srv__MusicPlay_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MusicPlay_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<MusicPlay_Response>) -> bool;
}

// Corresponds to audio_common_msgs__srv__MusicPlay_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MusicPlay_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for MusicPlay_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__srv__MusicPlay_Response__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__srv__MusicPlay_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MusicPlay_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__srv__MusicPlay_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__srv__MusicPlay_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__srv__MusicPlay_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MusicPlay_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MusicPlay_Response where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/srv/MusicPlay_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__srv__MusicPlay_Response() }
  }
}






#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__audio_common_msgs__srv__MusicPlay() -> *const std::ffi::c_void;
}

// Corresponds to audio_common_msgs__srv__MusicPlay
#[allow(missing_docs, non_camel_case_types)]
pub struct MusicPlay;

impl rosidl_runtime_rs::Service for MusicPlay {
    type Request = MusicPlay_Request;
    type Response = MusicPlay_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__audio_common_msgs__srv__MusicPlay() }
    }
}


