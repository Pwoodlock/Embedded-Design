////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//     This software is supplied under the terms of a license agreement or    //
//     nondisclosure agreement with Mitov Software and may not be copied      //
//     or disclosed except in accordance with the terms of that agreement.    //
//         Copyright(c) 2002-2018 Mitov Software. All Rights Reserved.        //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
//
//  Patrick Woodlock
//  25- 04 2018
//  Fixed code for MAXIM Thermocouple IC Bug in which read out on SPI 
//  bus required tweaking
//  
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _MITOV_MAXIM_THERMOCOUPLE_h
#define _MITOV_MAXIM_THERMOCOUPLE_h

#include <Mitov.h>

#ifdef __TEST_FOR_DEBUG_PRINTS__
#define Serial UNGUARDED DEBUG PRINT!!!
#endif

namespace Mitov
{
//---------------------------------------------------------------------------
	class NotConnectedPinBasicImplementation
	{
	public:
		bool	Enabled : 1;
		bool	InFahrenheit : 1;

	protected:
		bool	FDataSent : 1;
		bool	FNotConnected : 1;
		bool	FColdJunction : 1;
		bool	FShortToGroundOutputPin : 1;

	public:
		NotConnectedPinBasicImplementation() :
			Enabled( true ),
			InFahrenheit( false ),
			FDataSent( false ),
			FNotConnected( false ),
			FColdJunction( false ),
			FShortToGroundOutputPin( false )
		{
		}

	};
//---------------------------------------------------------------------------
	class NotConnectedPinImplementation : public NotConnectedPinBasicImplementation
	{
	public:
		OpenWire::SourcePin	NotConnectedOutputPin;

	protected:
		inline void SetNonConnected( bool AValue, bool AChangeOnly )
		{
			if( AChangeOnly )
				if( FNotConnected == AValue )
					return;

			FNotConnected = AValue;
			NotConnectedOutputPin.Notify( &AValue );
		}

	};
//---------------------------------------------------------------------------
	class NotConnectedPinNullImplementation : public NotConnectedPinBasicImplementation
	{
	protected:
		inline void SetNonConnected( bool AValue, bool AChangeOnly ) {}

	};
//---------------------------------------------------------------------------
	template<typename T_BASE> class ColdJunctionOutputPinImplementation : public T_BASE
	{
	public:
		OpenWire::TypedSourcePin<float>	ColdJunctionOutputPin;

	protected:
		inline void SetColdJunction( float AValue, bool AChangeOnly )
		{
			ColdJunctionOutputPin.SetValue( AValue, AChangeOnly );
		}

	};
//---------------------------------------------------------------------------
	template<typename T_BASE> class ColdJunctionOutputPinNullImplementation : public T_BASE
	{
	protected:
		inline void SetColdJunction( bool AValue, bool AChangeOnly ) {}

	};
//---------------------------------------------------------------------------
	template<typename T_BASE> class ShortToPowerOutputPinImplementation : public T_BASE
	{
	public:
		OpenWire::SourcePin	ShortToPowerOutputPin;

	protected:
		inline void SetShortToPower( bool AValue, bool AChangeOnly )
		{
			if( AChangeOnly )
				if( T_BASE::FShortToGround == AValue )
					return;

			T_BASE::FShortToGround = AValue;
			ShortToPowerOutputPin.Notify( &AValue );
		}

	};
//---------------------------------------------------------------------------
	template<typename T_BASE> class ShortToPowerOutputPinNullImplementation : public T_BASE
	{
	protected:
		inline void SetShortToPower( bool AValue, bool AChangeOnly ) {}

	};
//---------------------------------------------------------------------------
	template<typename T_BASE> class ShortToGroundOutputPinImplementation : public T_BASE
	{
	public:
		OpenWire::SourcePin	ShortToGroundOutputPin;

	protected:
		inline void SetShortToGround( bool AValue, bool AChangeOnly )
		{
			if( AChangeOnly )
				if( T_BASE::FShortToGround == AValue )
					return;

			T_BASE::FShortToGround = AValue;
			ShortToGroundOutputPin.Notify( &AValue );
		}

	};
//---------------------------------------------------------------------------
	template<typename T_BASE> class ShortToGroundOutputPinNullImplementation : public T_BASE
	{
	protected:
		inline void SetShortToGround( bool AValue, bool AChangeOnly ) {}

	};
//---------------------------------------------------------------------------
	template<typename T_SPI, T_SPI &C_SPI, typename T_CS_IMPLEMENTATION, typename T_NON_CONNECTED_IMPLEMENTATION> class ThermocoupleMAX6675 : public T_CS_IMPLEMENTATION, public T_NON_CONNECTED_IMPLEMENTATION
	{
	public:
		OpenWire::TypedSourcePin<float>	OutputPin;

	public:
		bool	Enabled = true;

	protected:
		void ReadSensor( bool AChangeOnly )
		{
			if( ! Enabled )
				return;

			T_CS_IMPLEMENTATION::SetChipSelect( false );

			C_SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

			uint16_t Avalue = uint16_t( C_SPI.transfer(0) ) << 8;
			Avalue |= C_SPI.transfer(0);

			C_SPI.endTransaction();

			T_CS_IMPLEMENTATION::SetChipSelect( true );

			float ATemperature;
			if ( Avalue & 0b100 )
			{
				ATemperature = 0;
				T_NON_CONNECTED_IMPLEMENTATION::SetNonConnected( true, AChangeOnly );
			}

			else
			{
				T_NON_CONNECTED_IMPLEMENTATION::SetNonConnected( false, AChangeOnly );
				Avalue >>= 3;
				if( T_NON_CONNECTED_IMPLEMENTATION::InFahrenheit )
					ATemperature = Avalue * 0.25 * 9.0 / 5.0 + 32;

				else
					ATemperature = Avalue * 0.25;

			}

			OutputPin.SetValue( ATemperature, AChangeOnly );
		}

	public:
		inline void ClockInputPin_o_Receive( void *_Data )
		{
			ReadSensor( false );
		}

	public:
		inline void SystemLoopBegin( unsigned long currentMicros )
		{
			ReadSensor( T_NON_CONNECTED_IMPLEMENTATION::FDataSent );
			T_NON_CONNECTED_IMPLEMENTATION::FDataSent = true;
		}

	};
//---------------------------------------------------------------------------
	template<typename T_SPI, T_SPI &C_SPI, typename T_CS_IMPLEMENTATION, typename T_NON_CONNECTED_IMPLEMENTATION> class ThermocoupleMAX31855 : public T_CS_IMPLEMENTATION, public T_NON_CONNECTED_IMPLEMENTATION
	{
	public:
		OpenWire::TypedSourcePin<float>	OutputPin;

	public:
		bool	Enabled = true;

	protected:
		void ReadSensor( bool AChangeOnly )
		{
			if( ! Enabled )
				return;

			T_CS_IMPLEMENTATION::SetChipSelect( false );

			C_SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

			uint16_t Avalue0 = uint16_t( C_SPI.transfer(0) ) << 8;
			Avalue0 |= uint32_t( C_SPI.transfer(0) );

			uint16_t Avalue1 = uint16_t( C_SPI.transfer(0) ) << 8;

			C_SPI.endTransaction();
			T_CS_IMPLEMENTATION::SetChipSelect( true );
			T_NON_CONNECTED_IMPLEMENTATION::SetNonConnected( ( Avalue1 & 0b001 ), AChangeOnly );
			T_NON_CONNECTED_IMPLEMENTATION::SetShortToGround( ( Avalue1 & 0b010 ), AChangeOnly );
			T_NON_CONNECTED_IMPLEMENTATION::SetShortToPower( ( Avalue1 & 0b100 ), AChangeOnly );
			float ATemperature;
			float AColdJunctionTemperature;
			if ( Avalue1 & 0b111 )
			{
				ATemperature = 0;
				AColdJunctionTemperature = 0;
			}

			else
			{
				if( T_NON_CONNECTED_IMPLEMENTATION::InFahrenheit ) 
				{
					ATemperature = ( Avalue0 & 0b1111111111111100 ) * ( 0.25 / 4 ) * 9.0 / 5.0 + 32; 
					AColdJunctionTemperature = ( Avalue1 & 0b1111111111110000 ) * ( 0.0625 / 16 ) * 9.0 / 5.0 + 32;
				}

				else
				{
					ATemperature = ( Avalue0 & 0b1111111111111100 ) * ( 0.25 / 4 );
					AColdJunctionTemperature = ( Avalue1 & 0b1111111111110000 ) * ( 0.0625 / 16 );
				}
			}

			OutputPin.SetValue( ATemperature, AChangeOnly );
			T_NON_CONNECTED_IMPLEMENTATION::SetColdJunction( AColdJunctionTemperature, AChangeOnly );
		}

	public:
		inline void ClockInputPin_o_Receive( void *_Data )
		{
			ReadSensor( false );
		}

	public:
		inline void SystemLoopBegin( unsigned long currentMicros )
		{
			ReadSensor( T_NON_CONNECTED_IMPLEMENTATION::FDataSent );
			T_NON_CONNECTED_IMPLEMENTATION::FDataSent = true;
		}

	};
//---------------------------------------------------------------------------
}

#ifdef __TEST_FOR_DEBUG_PRINTS__
#undef Serial
#endif

#endif
