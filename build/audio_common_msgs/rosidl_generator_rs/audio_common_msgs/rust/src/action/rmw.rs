
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_Goal() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_Goal__init(msg: *mut TTS_Goal) -> bool;
    fn audio_common_msgs__action__TTS_Goal__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_Goal>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_Goal__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_Goal>);
    fn audio_common_msgs__action__TTS_Goal__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_Goal>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_Goal>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_Goal
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_Goal {

    // This member is not documented.
    #[allow(missing_docs)]
    pub text: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub language: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub volume: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rate: f32,

}



impl Default for TTS_Goal {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_Goal__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_Goal__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_Goal {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Goal__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Goal__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Goal__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_Goal {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_Goal where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_Goal";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_Goal() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_Result() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_Result__init(msg: *mut TTS_Result) -> bool;
    fn audio_common_msgs__action__TTS_Result__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_Result>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_Result__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_Result>);
    fn audio_common_msgs__action__TTS_Result__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_Result>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_Result>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_Result
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_Result {

    // This member is not documented.
    #[allow(missing_docs)]
    pub text: rosidl_runtime_rs::String,

}



impl Default for TTS_Result {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_Result__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_Result__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_Result {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Result__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Result__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Result__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_Result {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_Result where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_Result";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_Result() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_Feedback() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_Feedback__init(msg: *mut TTS_Feedback) -> bool;
    fn audio_common_msgs__action__TTS_Feedback__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_Feedback>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_Feedback__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_Feedback>);
    fn audio_common_msgs__action__TTS_Feedback__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_Feedback>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_Feedback>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_Feedback
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_Feedback {

    // This member is not documented.
    #[allow(missing_docs)]
    pub audio: super::super::msg::rmw::AudioStamped,

}



impl Default for TTS_Feedback {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_Feedback__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_Feedback__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_Feedback {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Feedback__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Feedback__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_Feedback__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_Feedback {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_Feedback where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_Feedback";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_Feedback() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_FeedbackMessage() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_FeedbackMessage__init(msg: *mut TTS_FeedbackMessage) -> bool;
    fn audio_common_msgs__action__TTS_FeedbackMessage__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_FeedbackMessage>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_FeedbackMessage__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_FeedbackMessage>);
    fn audio_common_msgs__action__TTS_FeedbackMessage__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_FeedbackMessage>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_FeedbackMessage>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_FeedbackMessage
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_FeedbackMessage {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub feedback: super::super::action::rmw::TTS_Feedback,

}



impl Default for TTS_FeedbackMessage {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_FeedbackMessage__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_FeedbackMessage__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_FeedbackMessage {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_FeedbackMessage__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_FeedbackMessage__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_FeedbackMessage__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_FeedbackMessage {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_FeedbackMessage where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_FeedbackMessage";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_FeedbackMessage() }
  }
}




#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_SendGoal_Request() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_SendGoal_Request__init(msg: *mut TTS_SendGoal_Request) -> bool;
    fn audio_common_msgs__action__TTS_SendGoal_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_SendGoal_Request>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_SendGoal_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_SendGoal_Request>);
    fn audio_common_msgs__action__TTS_SendGoal_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_SendGoal_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_SendGoal_Request>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_SendGoal_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_SendGoal_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,


    // This member is not documented.
    #[allow(missing_docs)]
    pub goal: super::super::action::rmw::TTS_Goal,

}



impl Default for TTS_SendGoal_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_SendGoal_Request__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_SendGoal_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_SendGoal_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_SendGoal_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_SendGoal_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_SendGoal_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_SendGoal_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_SendGoal_Request where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_SendGoal_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_SendGoal_Request() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_SendGoal_Response() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_SendGoal_Response__init(msg: *mut TTS_SendGoal_Response) -> bool;
    fn audio_common_msgs__action__TTS_SendGoal_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_SendGoal_Response>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_SendGoal_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_SendGoal_Response>);
    fn audio_common_msgs__action__TTS_SendGoal_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_SendGoal_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_SendGoal_Response>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_SendGoal_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_SendGoal_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub accepted: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::rmw::Time,

}



impl Default for TTS_SendGoal_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_SendGoal_Response__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_SendGoal_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_SendGoal_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_SendGoal_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_SendGoal_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_SendGoal_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_SendGoal_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_SendGoal_Response where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_SendGoal_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_SendGoal_Response() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_GetResult_Request() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_GetResult_Request__init(msg: *mut TTS_GetResult_Request) -> bool;
    fn audio_common_msgs__action__TTS_GetResult_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_GetResult_Request>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_GetResult_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_GetResult_Request>);
    fn audio_common_msgs__action__TTS_GetResult_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_GetResult_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_GetResult_Request>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_GetResult_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_GetResult_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_id: unique_identifier_msgs::msg::rmw::UUID,

}



impl Default for TTS_GetResult_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_GetResult_Request__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_GetResult_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_GetResult_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_GetResult_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_GetResult_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_GetResult_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_GetResult_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_GetResult_Request where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_GetResult_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_GetResult_Request() }
  }
}


#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_GetResult_Response() -> *const std::ffi::c_void;
}

#[link(name = "audio_common_msgs__rosidl_generator_c")]
extern "C" {
    fn audio_common_msgs__action__TTS_GetResult_Response__init(msg: *mut TTS_GetResult_Response) -> bool;
    fn audio_common_msgs__action__TTS_GetResult_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TTS_GetResult_Response>, size: usize) -> bool;
    fn audio_common_msgs__action__TTS_GetResult_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TTS_GetResult_Response>);
    fn audio_common_msgs__action__TTS_GetResult_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TTS_GetResult_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<TTS_GetResult_Response>) -> bool;
}

// Corresponds to audio_common_msgs__action__TTS_GetResult_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TTS_GetResult_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub status: i8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub result: super::super::action::rmw::TTS_Result,

}



impl Default for TTS_GetResult_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !audio_common_msgs__action__TTS_GetResult_Response__init(&mut msg as *mut _) {
        panic!("Call to audio_common_msgs__action__TTS_GetResult_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TTS_GetResult_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_GetResult_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_GetResult_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { audio_common_msgs__action__TTS_GetResult_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TTS_GetResult_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TTS_GetResult_Response where Self: Sized {
  const TYPE_NAME: &'static str = "audio_common_msgs/action/TTS_GetResult_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__audio_common_msgs__action__TTS_GetResult_Response() }
  }
}






#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__audio_common_msgs__action__TTS_SendGoal() -> *const std::ffi::c_void;
}

// Corresponds to audio_common_msgs__action__TTS_SendGoal
#[allow(missing_docs, non_camel_case_types)]
pub struct TTS_SendGoal;

impl rosidl_runtime_rs::Service for TTS_SendGoal {
    type Request = TTS_SendGoal_Request;
    type Response = TTS_SendGoal_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__audio_common_msgs__action__TTS_SendGoal() }
    }
}




#[link(name = "audio_common_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__audio_common_msgs__action__TTS_GetResult() -> *const std::ffi::c_void;
}

// Corresponds to audio_common_msgs__action__TTS_GetResult
#[allow(missing_docs, non_camel_case_types)]
pub struct TTS_GetResult;

impl rosidl_runtime_rs::Service for TTS_GetResult {
    type Request = TTS_GetResult_Request;
    type Response = TTS_GetResult_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__audio_common_msgs__action__TTS_GetResult() }
    }
}


