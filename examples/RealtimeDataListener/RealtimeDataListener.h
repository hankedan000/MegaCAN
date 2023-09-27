#pragma once

#include <MegaCAN_Device.h>

struct RT_Data
{
  MegaCAN::RtMsg00_t m0;
  MegaCAN::RtMsg01_t m1;
  MegaCAN::RtMsg02_t m2;
  MegaCAN::RtMsg03_t m3;
  MegaCAN::RtMsg04_t m4;
  MegaCAN::RtMsg05_t m5;
  MegaCAN::RtMsg06_t m6;
  MegaCAN::RtMsg07_t m7;
  MegaCAN::RtMsg08_t m8;
  MegaCAN::RtMsg09_t m9;
  MegaCAN::RtMsg10_t m10;
  MegaCAN::RtMsg11_t m11;
  MegaCAN::RtMsg12_t m12;
  MegaCAN::RtMsg13_t m13;
  MegaCAN::RtMsg14_t m14;
  MegaCAN::RtMsg15_t m15;
  MegaCAN::RtMsg16_t m16;
  MegaCAN::RtMsg17_t m17;
  MegaCAN::RtMsg18_t m18;
  MegaCAN::RtMsg19_t m19;
  MegaCAN::RtMsg20_t m20;
  MegaCAN::RtMsg21_t m21;
  MegaCAN::RtMsg22_t m22;
  MegaCAN::RtMsg23_t m23;
  MegaCAN::RtMsg24_t m24;
  MegaCAN::RtMsg25_t m25;
  MegaCAN::RtMsg26_t m26;
  MegaCAN::RtMsg27_t m27;
  MegaCAN::RtMsg28_t m28;
  MegaCAN::RtMsg29_t m29;
  MegaCAN::RtMsg30_t m30;
  MegaCAN::RtMsg31_t m31;
  MegaCAN::RtMsg32_t m32;
  MegaCAN::RtMsg33_t m33;
  MegaCAN::RtMsg34_t m34;
  MegaCAN::RtMsg35_t m35;
  MegaCAN::RtMsg36_t m36;
  MegaCAN::RtMsg37_t m37;
  MegaCAN::RtMsg38_t m38;
  MegaCAN::RtMsg39_t m39;
  MegaCAN::RtMsg40_t m40;
  MegaCAN::RtMsg41_t m41;
  MegaCAN::RtMsg42_t m42;
  MegaCAN::RtMsg43_t m43;
  MegaCAN::RtMsg44_t m44;
  MegaCAN::RtMsg45_t m45;
  MegaCAN::RtMsg46_t m46;
  MegaCAN::RtMsg47_t m47;
  MegaCAN::RtMsg48_t m48;
  MegaCAN::RtMsg49_t m49;
  MegaCAN::RtMsg50_t m50;
  MegaCAN::RtMsg51_t m51;
  MegaCAN::RtMsg52_t m52;
  MegaCAN::RtMsg53_t m53;
  MegaCAN::RtMsg54_t m54;
  MegaCAN::RtMsg55_t m55;
  MegaCAN::RtMsg56_t m56;
  MegaCAN::RtMsg57_t m57;
  MegaCAN::RtMsg58_t m58;
  MegaCAN::RtMsg59_t m59;
  MegaCAN::RtMsg60_t m60;
  MegaCAN::RtMsg61_t m61;
  MegaCAN::RtMsg62_t m62;
};

// A class that listens for Megasaquirt realtime data messages. It stores
// and provides access to the most recent engine data received from the bus.
class RealtimeDataListener : public MegaCAN::Device
{
public:
	RealtimeDataListener(
			uint8_t cs,
			uint8_t myId,
			uint8_t intPin,
			MegaCAN::CAN_Msg *buff,
			uint8_t buffSize);

  const RT_Data &
  data() const
  {
    return data_;
  }

protected:
  // override so we can mark option to handle standard msgs immediately
	virtual void
	getOptions(
		struct MegaCAN::Options *opts) override;

  /**
   * override this base method so that we can set filters for broadcast
   * frame reception (11bit protocol)
   */
  virtual void
  applyCanFilters(
    MCP_CAN *can) override;
  
	/**
	 * Called when a standard 11bit megasquirt broadcast frame is received.
	 * 
	 * @param[in] id
	 * The 11bit CAN identifier
	 * 
	 * @param[in] length
	 * The number of data bytes in the CAN frame
	 * 
	 * @param[in] data
	 * A pointer to the data segment of the CAN frame
	 */
	virtual void
	handleStandard(
			const uint32_t id,
			const uint8_t length,
			uint8_t *data) override;

private:
  RT_Data data_;

};