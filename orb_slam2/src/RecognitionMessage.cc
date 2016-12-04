#include "RecognitionMessage.h"
#include <iostream>
#include <cstring>

namespace ORB_SLAM2
{
  RecognitionMessage::RecognitionMessage():flag(false),depth(0.0f),name(""){}
  RecognitionMessage::RecognitionMessage(bool _flag, float _depth, std::string _name):flag(_flag),depth(_depth),name(_name){}
  RecognitionMessage::RecognitionMessage(const RecognitionMessage &recog):flag(recog.flag),depth(recog.depth),name(recog.name){}
  RecognitionMessage::~RecognitionMessage(){}
  void RecognitionMessage::SetWorldPos(const std::vector<float> &Pos)
  {
  	// std::unique_lock<mutex> lock(GlobalMutex);
  	worldPos = Pos;
  }
  const std::vector<float> RecognitionMessage::GetWorldPos()
  {
  	// std::unique_lock<mutex> lock(GlobalMutex);
  	return worldPos;
  }
} //namespace ORB_SLAM