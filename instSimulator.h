//---------------------------------------------
// instSimulator.h
//---------------------------------------------

#pragma once
#include "boatSimulator.h"
#include <NMEA2000.h>

#define INST_DEPTH			0
#define INST_LOG			1
#define INST_WIND			2
#define INST_COMPASS		3
#define INST_GPS			4
#define INST_AUTOPILOT		5
#define INST_ENGINE 		6
#define INST_GENSET			7

#define NUM_INSTRUMENTS 	8

#define PROTOCOL_NONE 		0x00
#define PROTOCOL_SEATALK	0x01
#define PROTOCOL_0183		0x02
#define PROTOCOL_2000		0x04


//-------------------------------
// instBase
//-------------------------------

class instBase
{
public:

	instBase(int inst_num) : m_inst_num(inst_num) {}

	virtual void init(tNMEA2000 *nmea2000)
		{ m_nmea2000 = nmea2000; }

	int getInstNum()	{ return m_inst_num; };
	virtual const char *getInstName() = 0;

	void setProtocol(uint8_t protocol, bool on)
	{
		m_protocols &= ~protocol;
		if (on) m_protocols |= protocol;
	}

	virtual void sendProtocols() = 0;


protected:

	int m_inst_num;
	uint8_t m_protocols;

	tNMEA2000 *m_nmea2000;

};	// class instBase


//--------------------------------
// instruments
//--------------------------------

class depthInst : public instBase
	{
	public:

		depthInst() : instBase(INST_DEPTH) {};

	private:

		virtual const char *getInstName() override { return "DEPTH"; };
		virtual void sendProtocols() override;
	};


class logInst : public instBase
	{
	public:

		logInst() : instBase(INST_LOG) {}

	private:

		virtual const char *getInstName() override { return "LOG"; };
		virtual void sendProtocols() override;
	};


class windInst : public instBase
	{
	public:

		windInst() : instBase(INST_WIND) {}

	private:

		virtual const char *getInstName() override { return "WIND"; };
		virtual void sendProtocols() override;
	};


class compassInst : public instBase
	{
	public:

		compassInst() : instBase(INST_COMPASS) {}

	private:

		virtual const char *getInstName() override { return "WIND"; };
		virtual void sendProtocols() override;
	};


class gpsInst : public instBase
	{
	public:

		gpsInst() : instBase(INST_GPS) {}

	private:

		virtual const char *getInstName() override { return "GPS"; };
		virtual void sendProtocols() override;
	};
	

class autopilotInst : public instBase
	{
	public:

		autopilotInst() : instBase(INST_AUTOPILOT) {}

	private:

		virtual const char *getInstName() override { return "GPS"; };
		virtual void sendProtocols() override;
	};


class engineInst : public instBase
	{
	public:

		engineInst() : instBase(INST_ENGINE) {}

	private:

		virtual const char *getInstName() override { return "ENGINE"; };
		virtual void sendProtocols() override;
	};


class gensetInst : public instBase
	{
	public:

		gensetInst() : instBase(INST_GENSET) {}

	private:

		virtual const char *getInstName() override { return "GENSET"; };
		virtual void sendProtocols() override;
	};



//-------------------------------
// instSimulator
//-------------------------------

class instSimulator
{
public:
	
	void init(tNMEA2000 *nmea2000);
	void run();

	void setProtocol(int inst_num, uint8_t protocol, bool on)
	{
		m_inst[inst_num]->setProtocol(protocol,on);
	}

private:

	tNMEA2000 *m_nmea2000;

	instBase *m_inst[NUM_INSTRUMENTS];
	
};	// class instSimulator



extern instSimulator instruments;


// end of instSimulator.h
