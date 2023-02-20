#pragma once
#ifdef UseMockOkFrontPanel
#include <fmt/format.h>

#include "okFrontPanel.h"

using fmt::print;

class MockOkCFrontPanel : public okCFrontPanel
{
public:
    ErrorCode UpdateWireIns()
    {
        fmt::print("UpdateWireIns\n");
        return okCFrontPanel::UpdateWireIns();
    }
    ErrorCode GetWireInValue(int epAddr, UINT32 *val)
    {
        fmt::print("GetWireInValue 0x{:x}\n", epAddr);
        return okCFrontPanel::GetWireInValue(epAddr, val);
    }
    ErrorCode SetWireInValue(int ep, UINT32 val, UINT32 mask = 0xffffffff)
    {
        fmt::print("SetWireInValue 0x{:x} 0x{:x} 0x{:x}\n", ep, val, mask);
        return okCFrontPanel::SetWireInValue(ep, val, mask);
    }
    ErrorCode UpdateWireOuts()
    {
        fmt::print("UpdateWireOuts\n");
        return okCFrontPanel::UpdateWireOuts();
    }
    unsigned long GetWireOutValue(int epAddr)
    {
        fmt::print("GetWireOutValue 0x{:x}\n", epAddr);
        return okCFrontPanel::GetWireOutValue(epAddr);
    }
    ErrorCode ActivateTriggerIn(int epAddr, int bit)
    {
        fmt::print("ActivateTriggerIn 0x{:x} 0x{:x}\n", epAddr, bit);
        return okCFrontPanel::ActivateTriggerIn(epAddr, bit);
    }
    long WriteToPipeIn(int epAddr, long length, unsigned char *data)
    {
        fmt::print("WriteToPipeIn 0x{:x} 0x{:x}\n", epAddr, length);
        return okCFrontPanel::WriteToPipeIn(epAddr, length, data);
    }
    long ReadFromPipeOut(int epAddr, long length, unsigned char *data)
    {
        fmt::print("ReadFromPipeOut 0x{:x} 0x{:x} 0x{:x}\n", epAddr, length);
        return okCFrontPanel::ReadFromPipeOut(epAddr, length, data);
    }
    long WriteToBlockPipeIn(int epAddr, int blockSize, long length, unsigned char *data)
    {
        fmt::print("WriteToBlockPipeIn 0x{:x} 0x{:x} 0x{:x}\n", epAddr, blockSize, length);
        return okCFrontPanel::WriteToBlockPipeIn(epAddr, blockSize, length, data);
    }
    long ReadFromBlockPipeOut(int epAddr, int blockSize, long length, unsigned char *data)
    {
        fmt::print("ReadFromBlockPipeOut 0x{:x} 0x{:x} 0x{:x}\n", epAddr, blockSize, length);
        return okCFrontPanel::ReadFromBlockPipeOut(epAddr, blockSize, length, data);
    }
};
#endif