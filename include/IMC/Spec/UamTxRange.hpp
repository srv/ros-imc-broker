//***************************************************************************
// Copyright 2017 OceanScan - Marine Systems & Technology, Lda.             *
//***************************************************************************
// Licensed under the Apache License, Version 2.0 (the "License");          *
// you may not use this file except in compliance with the License.         *
// You may obtain a copy of the License at                                  *
//                                                                          *
// http://www.apache.org/licenses/LICENSE-2.0                               *
//                                                                          *
// Unless required by applicable law or agreed to in writing, software      *
// distributed under the License is distributed on an "AS IS" BASIS,        *
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
// See the License for the specific language governing permissions and      *
// limitations under the License.                                           *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************
// Automatically generated.                                                 *
//***************************************************************************
// IMC XML MD5: 4a9faba4a5957552c96963d5ea855de1                            *
//***************************************************************************

#ifndef IMC_UAMTXRANGE_HPP_INCLUDED_
#define IMC_UAMTXRANGE_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ostream>
#include <string>
#include <vector>

// IMC headers.
#include <IMC/Base/Config.hpp>
#include <IMC/Base/Message.hpp>
#include <IMC/Base/InlineMessage.hpp>
#include <IMC/Base/MessageList.hpp>
#include <IMC/Base/JSON.hpp>
#include <IMC/Base/Serialization.hpp>
#include <IMC/Spec/Enumerations.hpp>
#include <IMC/Spec/Bitfields.hpp>

namespace IMC
{
  //! UamTxRange.
  class UamTxRange: public Message
  {
  public:
    //! Sequence Id.
    uint16_t seq;
    //! Destination System.
    std::string sys_dst;
    //! Timeout.
    float timeout;

    static uint16_t
    getIdStatic(void)
    {
      return 818;
    }

    static UamTxRange*
    cast(Message* msg__)
    {
      return (UamTxRange*)msg__;
    }

    UamTxRange(void)
    {
      m_header.mgid = UamTxRange::getIdStatic();
      clear();
    }

    UamTxRange*
    clone(void) const
    {
      return new UamTxRange(*this);
    }

    void
    clear(void)
    {
      seq = 0;
      sys_dst.clear();
      timeout = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UamTxRange& other__ = static_cast<const UamTxRange&>(msg__);
      if (seq != other__.seq) return false;
      if (sys_dst != other__.sys_dst) return false;
      if (timeout != other__.timeout) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(seq, ptr__);
      ptr__ += IMC::serialize(sys_dst, ptr__);
      ptr__ += IMC::serialize(timeout, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(seq, bfr__, size__);
      bfr__ += IMC::deserialize(sys_dst, bfr__, size__);
      bfr__ += IMC::deserialize(timeout, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(seq, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sys_dst, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(timeout, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UamTxRange::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UamTxRange";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 6;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(sys_dst);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "seq", seq, nindent__);
      IMC::toJSON(os__, "sys_dst", sys_dst, nindent__);
      IMC::toJSON(os__, "timeout", timeout, nindent__);
    }
  };
}

#endif