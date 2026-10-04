#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to audio_common_msgs__srv__MusicPlay_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MusicPlay_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub audio: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub file_path: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub loop_: bool,

}



impl Default for MusicPlay_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::MusicPlay_Request::default())
  }
}

impl rosidl_runtime_rs::Message for MusicPlay_Request {
  type RmwMsg = super::srv::rmw::MusicPlay_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        audio: msg.audio.as_str().into(),
        file_path: msg.file_path.as_str().into(),
        loop_: msg.loop_,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        audio: msg.audio.as_str().into(),
        file_path: msg.file_path.as_str().into(),
      loop_: msg.loop_,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      audio: msg.audio.to_string(),
      file_path: msg.file_path.to_string(),
      loop_: msg.loop_,
    }
  }
}


// Corresponds to audio_common_msgs__srv__MusicPlay_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MusicPlay_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,

}



impl Default for MusicPlay_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::MusicPlay_Response::default())
  }
}

impl rosidl_runtime_rs::Message for MusicPlay_Response {
  type RmwMsg = super::srv::rmw::MusicPlay_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
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


