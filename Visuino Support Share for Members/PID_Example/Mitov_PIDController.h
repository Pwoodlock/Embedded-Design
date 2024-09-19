////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//     This software is supplied under the terms of a license agreement or    //
//     nondisclosure agreement with Mitov Software and may not be copied      //
//     or disclosed except in accordance with the terms of that agreement.    //
//         Copyright(c) 2002-2018 Mitov Software. All Rights Reserved.        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#ifndef _MITOV_PID_CONTROLLER_h
#define _MITOV_PID_CONTROLLER_h

#include <Mitov.h>

namespace Mitov
{
	class PIDController
	{
	public:
		OpenWire::SourcePin	OutputPin;

    public:
		bool	ClockInputPin_o_IsConnected : 1;

    public:
        bool	Enabled : 1;
		float	ProportionalGain = 0.1f;
		float	IntegralGain = 0.1f;
		float	DerivativeGain = 0.1f;

		float	SetPoint = 0.0f;

		float	InitialValue = 0.0f;

	protected:
		unsigned long	FLastTime;
		float	FOutput = 0.0f;
		float	FLastInput;
		float	FITerm;

	public:
		void SetEnabled( bool AValue )
		{
            if( Enabled == AValue )
                return;

            Enabled = AValue;
			if( Enabled )
				Initialize();

		}

	public:
		inline void InputPin_o_Receive( void *_Data )
		{
			InitialValue = *(float *)_Data;
		}

		void ManualControlInputPin_o_Receive( void *_Data )
		{
			if( Enabled )
				return;

			FOutput = *(float *)_Data;
		}

		inline void ClockInputPin_o_Receive( void *_Data )
		{
			OutputPin.Notify( &FOutput );
		}

	protected:
		void Initialize()
		{
			FITerm = FOutput;
			FLastInput = InitialValue;
			FLastTime = micros();
		}

	public:
		inline void SystemStart()
		{
			Initialize();
		}

		inline void SystemLoopBegin( unsigned long currentMicros )
		{
			if( ! Enabled ) 
				return;

			unsigned long timeChange = ( currentMicros - FLastTime );
			float ANormalizedTime = float( timeChange ) / 1000000;

			// Compute all the working error variables
			double error = SetPoint - InitialValue;
//			ITerm += ( ki * error ) * ANormalizedTime;
//			Serial.println( FITerm + ( IntegralGain * error ) * ANormalizedTime );

			FITerm = constrain( FITerm + ( IntegralGain * error ) * ANormalizedTime, -10000.0f, 10000.0f );
//			FITerm += ( IntegralGain * error ) * ANormalizedTime;

			double dInput = ( InitialValue - FLastInput ) * ANormalizedTime;
 
			// Compute PID Output
			float AOutput = constrain( ProportionalGain * error + FITerm - DerivativeGain * dInput, -1.5f, 1.0f ); // 0.0f
	  
			// Remember some variables for next time
			FLastInput = InitialValue;
			FLastTime = currentMicros;

			if( AOutput == FOutput )
				return;

			FOutput = AOutput;

			if( ClockInputPin_o_IsConnected )
				return;

			OutputPin.Notify( &FOutput );
		}

	public:
		PIDController() :
			ClockInputPin_o_IsConnected( false ),
			Enabled( true )
		{
		}

	};
//---------------------------------------------------------------------------
}

#endif
