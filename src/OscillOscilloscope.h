/***********************************************************************************************************************
*                                                                                                                      *
* Oscill DSO1 plugin for ngscopeclient / libscopehal                                                                   *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef OscillOscilloscope_h
#define OscillOscilloscope_h

#include "OscillProtocol.h"
#include "OscillSerial.h"

class EdgeTrigger;

/**
	@brief Driver for the OKB ROSA "Oscill" DSO1 single channel pen oscilloscope (CP210x USB-UART, OBEX framing)

	Connection: driver "oscill", transport "uart", path "/dev/ttyUSB0:921600" (handshake at 9600, then the driver
	switches the probe to the requested rate; confirmed on hardware, ~0.04 s per record vs ~0.33 s at 9600).
 */
class OscillOscilloscope : public virtual SCPIOscilloscope
{
public:
	OscillOscilloscope(SCPITransport* transport);
	virtual ~OscillOscilloscope();

	OscillOscilloscope(const OscillOscilloscope& rhs) =delete;
	OscillOscilloscope& operator=(const OscillOscilloscope& rhs) =delete;

	//Device information
	virtual std::string IDPing() override;
	virtual unsigned int GetInstrumentTypes() const override;
	virtual uint32_t GetInstrumentTypesForChannel(size_t i) const override;

	//Channel configuration
	virtual bool IsChannelEnabled(size_t i) override;
	virtual void EnableChannel(size_t i) override;
	virtual void DisableChannel(size_t i) override;
	virtual OscilloscopeChannel::CouplingType GetChannelCoupling(size_t i) override;
	virtual void SetChannelCoupling(size_t i, OscilloscopeChannel::CouplingType type) override;
	virtual std::vector<OscilloscopeChannel::CouplingType> GetAvailableCouplings(size_t i) override;
	virtual double GetChannelAttenuation(size_t i) override;
	virtual void SetChannelAttenuation(size_t i, double atten) override;
	virtual std::vector<unsigned int> GetChannelBandwidthLimiters(size_t i) override;
	virtual unsigned int GetChannelBandwidthLimit(size_t i) override;
	virtual void SetChannelBandwidthLimit(size_t i, unsigned int limit_mhz) override;
	virtual float GetChannelVoltageRange(size_t i, size_t stream) override;
	virtual void SetChannelVoltageRange(size_t i, size_t stream, float range) override;
	virtual OscilloscopeChannel* GetExternalTrigger() override;
	virtual float GetChannelOffset(size_t i, size_t stream) override;
	virtual void SetChannelOffset(size_t i, size_t stream, float offset) override;
	virtual bool IsHighRateOffsetCapable(size_t i) override;

	//Triggering
	virtual Oscilloscope::TriggerMode PollTrigger() override;
	virtual bool AcquireData() override;
	virtual void Start() override;
	virtual void StartSingleTrigger() override;
	virtual void Stop() override;
	virtual void ForceTrigger() override;
	virtual bool IsTriggerArmed() override;
	virtual bool HasAutoTrigger() override;
	virtual void StartAutoTrigger() override;
	virtual void PushTrigger() override;
	virtual void PullTrigger() override;
	virtual std::vector<std::string> GetTriggerTypes() override;

	//Timebase
	virtual std::vector<uint64_t> GetSampleRatesNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleRatesInterleaved() override;
	virtual std::set<InterleaveConflict> GetInterleaveConflicts() override;
	virtual std::vector<uint64_t> GetSampleDepthsNonInterleaved() override;
	virtual std::vector<uint64_t> GetSampleDepthsInterleaved() override;
	virtual uint64_t GetSampleRate() override;
	virtual uint64_t GetSampleDepth() override;
	virtual void SetSampleDepth(uint64_t depth) override;
	virtual void SetSampleRate(uint64_t rate) override;
	virtual void SetTriggerOffset(int64_t offset) override;
	virtual int64_t GetTriggerOffset() override;
	virtual bool IsInterleaving() override;
	virtual bool SetInterleaving(bool combine) override;

	//Input filters (O1 bits 2-3), shown as ADC modes
	virtual bool IsADCModeConfigurable() override;
	virtual bool IsADCModePerChannel() override;
	virtual std::vector<std::string> GetADCModeNames(size_t channel) override;
	virtual size_t GetADCMode(size_t channel) override;
	virtual void SetADCMode(size_t channel, size_t mode) override;

protected:
	//Low level protocol
	bool Connect();
	bool EnsureConnected();
	bool SendFrame(const oscill::Bytes& frame);
	bool ReadFrame(oscill::Bytes& frame, int firstByteTimeoutMs, int bodyTimeoutMs = 3000);
	bool Transact(const oscill::Bytes& request, oscill::Bytes& reply, int timeoutMs = 1500);
	bool ReadRegister(const std::string& name, oscill::RegValue& value);
	bool WriteRegister(const std::string& name, const oscill::Bytes& request);
	bool WriteReg1(const std::string& name, uint8_t value);
	bool WriteReg2(const std::string& name, uint16_t value);
	bool WriteReg4(const std::string& name, uint32_t value);
	bool ChangeBaudRate(unsigned int baud);
	void CancelPendingAcquisition();

	//Pushing cached configuration to hardware
	void PushAll();
	void PushCoupling();
	void PushFilter(size_t mode);
	void PushVerticalRange();
	void PushTimebase();
	void PushTriggerLevel();
	void PushTriggerSlope();
	void PushTriggerDelay();
	void PushAutoTimeout();
	bool ApplySoftwareTrigger(const oscill::WaveformFrame& wf);
	bool PushRunMode(uint8_t mode);

	//Scaling helpers
	double GetVoltsPerCode();
	int64_t GetSamplePeriodFs();
	double TSToRate(uint32_t ts);
	uint32_t RateToTS(uint64_t rate);
	EdgeTrigger* GetEdgeTrigger();

	OscillSerial m_port;
	bool m_connected;
	std::string m_idString;

	//Acquisition state machine
	bool m_requestOutstanding;
	bool m_frameReady;
	bool m_autoMode;
	bool m_forceNext;
	oscill::Bytes m_rxBuf;
	oscill::Bytes m_frame;
	double m_requestTime;
	int m_currentRunMode;

	//Cached configuration
	bool m_channelEnabled;
	OscilloscopeChannel::CouplingType m_coupling;
	double m_attenuation;
	float m_offset;
	size_t m_vrangeIndex;
	uint32_t m_mc;			//MCU clock period, 10 ps units
	uint32_t m_tsMin;		//TMl
	uint32_t m_tsMax;		//TMh
	uint32_t m_ts;			//current TS (clocks per sample * 256)
	uint64_t m_sampleRate;	//nominal rate requested by the user
	uint32_t m_tcMax;		//TCh, maximum pre-trigger samples
	uint8_t m_o1;			//input option bits
	uint8_t m_t1;			//trigger configuration bits
	int64_t m_triggerOffset;
	int64_t m_swTriggerPhase;	//time shift putting the software trigger crossing at m_triggerOffset
	uint64_t m_lastDepth;

	//Diagnostics: if the probe rejects register writes we stop spamming the log
	unsigned int m_regWriteFailures;

public:
	static std::string GetDriverNameInternal();
	static std::vector<SCPIInstrumentModel> GetDriverSupportedModels()
	{
		return
		{
			{"Oscill DSO1", {{ SCPITransportType::TRANSPORT_UART, "/dev/ttyUSB0:921600" }}}
		};
	}
	OSCILLOSCOPE_INITPROC(OscillOscilloscope)
};

#endif
