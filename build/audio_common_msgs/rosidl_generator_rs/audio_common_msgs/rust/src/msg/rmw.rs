#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__AudioData() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__msg__AudioData__init(msg: *mut AudioData) -> bool;
    fn audio_common_msgs__msg__AudioData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AudioData>, size: usize) -> bool;
    fn audio_common_msgs__msg__AudioData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AudioData>);
    fn audio_common_msgs__msg__AudioData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AudioData>, out_seq: *mut rosidl_runtime_rs::Sequence<AudioData>) -> bool;
}

// Corresponds to audio_common_msgs__msg__AudioData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AudioData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub float32_data: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub int32_data: rosidl_runtime_rs::Sequence<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub int16_data: rosidl_runtime_rs::Sequence<i16>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub int8_data: rosidl_runtime_rs::Sequence<i8>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uint8_data: rosidl_runtime_rs::Sequence<u8>,

}



impl Default for AudioData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__msg__AudioData__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__msg__AudioData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AudioData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AudioData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AudioData where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/msg/AudioData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__AudioData() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__AudioInfo() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__msg__AudioInfo__init(msg: *mut AudioInfo) -> bool;
    fn audio_common_msgs__msg__AudioInfo__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AudioInfo>, size: usize) -> bool;
    fn audio_common_msgs__msg__AudioInfo__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AudioInfo>);
    fn audio_common_msgs__msg__AudioInfo__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AudioInfo>, out_seq: *mut rosidl_runtime_rs::Sequence<AudioInfo>) -> bool;
}

// Corresponds to audio_common_msgs__msg__AudioInfo
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AudioInfo {

    // This member is not documented.
    #[allow(missing_docs)]
    pub format: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub channels: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rate: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub chunk: i32,

}



impl Default for AudioInfo {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__msg__AudioInfo__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__msg__AudioInfo__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AudioInfo {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioInfo__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioInfo__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioInfo__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AudioInfo {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AudioInfo where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/msg/AudioInfo";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__AudioInfo() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__Audio() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__msg__Audio__init(msg: *mut Audio) -> bool;
    fn audio_common_msgs__msg__Audio__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Audio>, size: usize) -> bool;
    fn audio_common_msgs__msg__Audio__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Audio>);
    fn audio_common_msgs__msg__Audio__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Audio>, out_seq: *mut rosidl_runtime_rs::Sequence<Audio>) -> bool;
}

// Corresponds to audio_common_msgs__msg__Audio
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Audio {

    // This member is not documented.
    #[allow(missing_docs)]
    pub audio_data: super::super::msg::rmw::AudioData,


    // This member is not documented.
    #[allow(missing_docs)]
    pub info: super::super::msg::rmw::AudioInfo,

}



impl Default for Audio {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__msg__Audio__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__msg__Audio__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Audio {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__Audio__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__Audio__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__Audio__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Audio {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Audio where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/msg/Audio";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__Audio() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__AudioStamped() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__msg__AudioStamped__init(msg: *mut AudioStamped) -> bool;
    fn audio_common_msgs__msg__AudioStamped__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AudioStamped>, size: usize) -> bool;
    fn audio_common_msgs__msg__AudioStamped__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AudioStamped>);
    fn audio_common_msgs__msg__AudioStamped__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AudioStamped>, out_seq: *mut rosidl_runtime_rs::Sequence<AudioStamped>) -> bool;
}

// Corresponds to audio_common_msgs__msg__AudioStamped
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AudioStamped {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub audio: super::super::msg::rmw::Audio,

}



impl Default for AudioStamped {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__msg__AudioStamped__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__msg__AudioStamped__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AudioStamped {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioStamped__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioStamped__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__msg__AudioStamped__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AudioStamped {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AudioStamped where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/msg/AudioStamped";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__msg__AudioStamped() }
  }
}


