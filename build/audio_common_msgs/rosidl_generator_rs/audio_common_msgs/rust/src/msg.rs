#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to audio_common_msgs__msg__AudioData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AudioData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub float32_data: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub int32_data: Vec<i32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub int16_data: Vec<i16>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub int8_data: Vec<i8>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub uint8_data: Vec<u8>,

}



impl Default for AudioData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AudioData::default())
  }
}

impl rosidl_runtime_rs::Message for AudioData {
  type RmwMsg = super::msg::rmw::AudioData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        float32_data: msg.float32_data.into(),
        int32_data: msg.int32_data.into(),
        int16_data: msg.int16_data.into(),
        int8_data: msg.int8_data.into(),
        uint8_data: msg.uint8_data.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        float32_data: msg.float32_data.as_slice().into(),
        int32_data: msg.int32_data.as_slice().into(),
        int16_data: msg.int16_data.as_slice().into(),
        int8_data: msg.int8_data.as_slice().into(),
        uint8_data: msg.uint8_data.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      float32_data: msg.float32_data
          .into_iter()
          .collect(),
      int32_data: msg.int32_data
          .into_iter()
          .collect(),
      int16_data: msg.int16_data
          .into_iter()
          .collect(),
      int8_data: msg.int8_data
          .into_iter()
          .collect(),
      uint8_data: msg.uint8_data
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to audio_common_msgs__msg__AudioInfo

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AudioInfo::default())
  }
}

impl rosidl_runtime_rs::Message for AudioInfo {
  type RmwMsg = super::msg::rmw::AudioInfo;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        format: msg.format,
        channels: msg.channels,
        rate: msg.rate,
        chunk: msg.chunk,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      format: msg.format,
      channels: msg.channels,
      rate: msg.rate,
      chunk: msg.chunk,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      format: msg.format,
      channels: msg.channels,
      rate: msg.rate,
      chunk: msg.chunk,
    }
  }
}


// Corresponds to audio_common_msgs__msg__Audio

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Audio {

    // This member is not documented.
    #[allow(missing_docs)]
    pub audio_data: super::msg::AudioData,


    // This member is not documented.
    #[allow(missing_docs)]
    pub info: super::msg::AudioInfo,

}



impl Default for Audio {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Audio::default())
  }
}

impl rosidl_runtime_rs::Message for Audio {
  type RmwMsg = super::msg::rmw::Audio;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        audio_data: super::msg::AudioData::into_rmw_message(std::borrow::Cow::Owned(msg.audio_data)).into_owned(),
        info: super::msg::AudioInfo::into_rmw_message(std::borrow::Cow::Owned(msg.info)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        audio_data: super::msg::AudioData::into_rmw_message(std::borrow::Cow::Borrowed(&msg.audio_data)).into_owned(),
        info: super::msg::AudioInfo::into_rmw_message(std::borrow::Cow::Borrowed(&msg.info)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      audio_data: super::msg::AudioData::from_rmw_message(msg.audio_data),
      info: super::msg::AudioInfo::from_rmw_message(msg.info),
    }
  }
}


// Corresponds to audio_common_msgs__msg__AudioStamped

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AudioStamped {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub audio: super::msg::Audio,

}



impl Default for AudioStamped {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AudioStamped::default())
  }
}

impl rosidl_runtime_rs::Message for AudioStamped {
  type RmwMsg = super::msg::rmw::AudioStamped;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        audio: super::msg::Audio::into_rmw_message(std::borrow::Cow::Owned(msg.audio)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        audio: super::msg::Audio::into_rmw_message(std::borrow::Cow::Borrowed(&msg.audio)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      audio: super::msg::Audio::from_rmw_message(msg.audio),
    }
  }
}


