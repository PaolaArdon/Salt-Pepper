/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef RECOGNITION_MESSAGE_H
#define RECOGNITION_MESSAGE_H

#include <iostream>
#include <cstring>
#include <opencv2/core/core.hpp>
// #include <mutex>

namespace ORB_SLAM2
{
  class RecognitionMessage
  {
    public:
      RecognitionMessage();
      RecognitionMessage(bool _flag, float _depth, std::string _name);
      RecognitionMessage(const RecognitionMessage &recog);
      void SetWorldPos(const std::vector<float> &Pos);
      const std::vector<float> GetWorldPos();
      bool flag;
      float depth;
      std::string name;
      std::vector<float> worldPos;
      ~RecognitionMessage();

      bool operator < (const RecognitionMessage &c) const
      {
        // std::cout << this->name << " < " << c.name << " == " << (this->name.compare(c.name) < 0) << std::endl;
        return (this->name.compare(c.name) < 0);
      }
      // bool operator == (const RecognitionMessage &c) const
      // {
      //   // std::cout << this->name << " < " << c.name << " == " << (this->name.compare(c.name) < 0) << std::endl;
      //   return (this->name.compare(c.name) == 0);
      // }
      // static std::mutex GlobalMutex;
  };
}
#endif // RECOGNITION_MESSAGE_H
