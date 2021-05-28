// -----------------------------------------------------------------------------
//    ASPiK Plugin Kernel File:  plugincore.cpp
//
/**
    \file   plugincore.cpp
    \author Will Pirkle
    \date   17-September-2018
    \brief  Implementation file for PluginCore object
    		- http://www.aspikplugins.com
    		- http://www.willpirkle.com
*/
// -----------------------------------------------------------------------------
#include "plugincore.h"
#include "plugindescription.h"

/**
\brief PluginCore constructor is launching pad for object initialization

Operations:
- initialize the plugin description (strings, codes, numbers, see initPluginDescriptors())
- setup the plugin's audio I/O channel support
- create the PluginParameter objects that represent the plugin parameters (see FX book if needed)
- create the presets
*/
PluginCore::PluginCore()
{
    // --- describe the plugin; call the helper to init the static parts you setup in plugindescription.h
    initPluginDescriptors();

    // --- default I/O combinations
	// --- for FX plugins
	if (getPluginType() == kFXPlugin)
	{
		addSupportedIOCombination({ kCFMono, kCFMono });
		addSupportedIOCombination({ kCFMono, kCFStereo });
		addSupportedIOCombination({ kCFStereo, kCFStereo });
	}
	else // --- synth plugins have no input, only output
	{
		addSupportedIOCombination({ kCFNone, kCFMono });
		addSupportedIOCombination({ kCFNone, kCFStereo });
	}

	// --- for sidechaining, we support mono and stereo inputs; auxOutputs reserved for future use
	addSupportedAuxIOCombination({ kCFMono, kCFNone });
	addSupportedAuxIOCombination({ kCFStereo, kCFNone });

	// --- create the parameters
    initPluginParameters();

    // --- create the presets
    initPluginPresets();
}

/**
\brief create all of your plugin parameters here

\return true if parameters were created, false if they already existed
*/
bool PluginCore::initPluginParameters()
{
	if (pluginParameterMap.size() > 0)
		return false;

    // --- Add your plugin parameter instantiation code bewtween these hex codes
	// **--0xDEA7--**
	PluginParameter* piParam = nullptr;

	piParam = new PluginParameter(controlID::modType, "Type", "XOR,AND,OR", "XOR");
	piParam->setBoundVariable(&modType, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::modLevel, "Level", "dB", controlVariableType::kDouble, -90.000000, 0.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&modLevel, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::mix, "Dry/Wet Mix", "%", controlVariableType::kDouble, 0.000000, 1.000000, 0.500000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&mix, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::phaseFlip, "Phase Flip", "NONE,PRE,POST,BOTH", "NONE");
	piParam->setBoundVariable(&phaseFlip, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::lagSamples, "Lag Samples", "", controlVariableType::kInt, 0.000000, 1000.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&lagSamples, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::multiFilterPosition, "Multi Filter Position", "OFF,PRE,POST", "OFF");
	piParam->setBoundVariable(&multiFilterPosition, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::lpfCutoff, "Multi Filter LP Cutoff", "Hz", controlVariableType::kDouble, 20.000000, 20480.000000, 1000.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&lpfCutoff, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::lpfQ, "Multi Filter LP Q", "", controlVariableType::kDouble, 0.707, 20.000000, 0.707, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&lpfQ, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::lpfMix, "LPF Mix", "%", controlVariableType::kDouble, 0.000000, 1.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&lpfMix, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::hpfMix, "HPF Mix", "%", controlVariableType::kDouble, 0.000000, 1.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&hpfMix, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(controlID::apfMix, "APF Mix", "%", controlVariableType::kDouble, 0.000000, 1.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(20.000000);
	piParam->setBoundVariable(&apfMix, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// **--0xEDA5--**
   
    // --- BONUS Parameter
    // --- SCALE_GUI_SIZE
    PluginParameter* piParamBonus = new PluginParameter(SCALE_GUI_SIZE, "Scale GUI", "tiny,small,medium,normal,large,giant", "normal");
    addPluginParameter(piParamBonus);

	// --- create the super fast access array
	initPluginParameterArray();

    return true;
}

/**
\brief initialize object for a new run of audio; called just before audio streams

Operation:
- store sample rate and bit depth on audioProcDescriptor - this information is globally available to all core functions
- reset your member objects here

\param resetInfo structure of information about current audio format

\return true if operation succeeds, false otherwise
*/
bool PluginCore::reset(ResetInfo& resetInfo)
{
    // --- save for audio processing
    audioProcDescriptor.sampleRate = resetInfo.sampleRate;
    audioProcDescriptor.bitDepth = resetInfo.bitDepth;

	buffer_L.reset(new double[bufferLength]);
	memset(&buffer_L[0], 0, bufferLength * sizeof(double));
	buffer_R.reset(new double[bufferLength]);
	memset(&buffer_R[0], 0, bufferLength * sizeof(double));
	writeIndex = 0;

    // --- other reset inits
    return PluginBase::reset(resetInfo);
}

/**
\brief one-time initialize function called after object creation and before the first reset( ) call

Operation:
- saves structure for the plugin to use; you can also load WAV files or state information here
*/
bool PluginCore::initialize(PluginInfo& pluginInfo)
{
	// --- add one-time init stuff here

	return true;
}

/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- syncInBoundVariables when preProcessAudioBuffers is called, it is *guaranteed* that all GUI control change information
  has been applied to plugin parameters; this binds parameter changes to your underlying variables
- NOTE: postUpdatePluginParameter( ) will be called for all bound variables that are acutally updated; if you need to process
  them individually, do so in that function
- use this function to bulk-transfer the bound variable data into your plugin's member object variables

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::preProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
    // --- sync internal variables to GUI parameters; you can also do this manually if you don't
    //     want to use the auto-variable-binding
    syncInBoundVariables();

    return true;
}

inline std::pair<long long, int> ifrexp(double d)
{
	int exp;
	double mant = frexp(d, &exp);
	return std::make_pair(static_cast<long long>((mant * pow(2.0, DBL_MANT_DIG))), static_cast<int>(exp));
}

inline double double_and_(double a, double b)
{
	auto pa = ifrexp(a);
	long long ma = pa.first;
	int ea = pa.second;
	auto pb = ifrexp(b);
	long long mb = pb.first;
	int eb = pb.second;

	mb = mb >> (ea - eb);

	if (ma < 0)
	{
		return (mb & ~(-ma)) * pow(2.0, ea - DBL_MANT_DIG);
	}
	if (mb < 0)
	{
		return (~(-mb) & ma) * pow(2.0, ea - DBL_MANT_DIG);
	}
	return (mb & ma) * pow(2.0, ea - DBL_MANT_DIG);
}

inline double double_or_(double a, double b)
{
	auto pa = ifrexp(a);
	long long ma = pa.first;
	int ea = pa.second;
	auto pb = ifrexp(b);
	long long mb = pb.first;
	int eb = pb.second;

	mb = mb >> (ea - eb);

	if (ma < 0)
	{
		return (-(~(mb | ~(-ma)))) * pow(2.0, ea - DBL_MANT_DIG);
	}
	if (mb < 0)
	{
		return (-(~(~(-mb) | ma))) * pow(2.0, ea - DBL_MANT_DIG);
	}
	return (mb | ma) * pow(2.0, ea - DBL_MANT_DIG);
}

inline double double_xor_(double a, double b)
{
	auto pa = ifrexp(a);
	long long ma = pa.first;
	int ea = pa.second;
	auto pb = ifrexp(b);
	long long mb = pb.first;
	int eb = pb.second;

	mb = mb >> (ea - eb);

	return (mb ^ ma) * pow(2.0, ea - DBL_MANT_DIG);
}

inline bool checkFloatUnderflow(double& value)
{
	bool retValue = false;
	if (value > 0.0 && value < 1.175494351e-38)
	{
		value = 0;
		retValue = true;
	}
	else if (value < 0.0 && value > -1.175494351e-38)
	{
		value = 0;
		retValue = true;
	}
	return retValue;
}

double double_and(double a, double b);
double double_or(double a, double b);
double double_xor(double a, double b);

double double_and(double a, double b)
{
	if (a == 0.0)
	{
		if (std::copysign(1.0, a) == 1.0)
		{
			return 0.0;
		}
		return b;
	}
	if (b == 0.0)
	{
		return double_and(b, a);
	}
	if (a < 0.0 && b < 0.0)
	{
		return -double_or(-a, -b);
	}
	if (fabs(a) >= fabs(b))
	{
		return double_and_(a, b);
	}
	return double_and_(b, a);
}

double double_or(double a, double b)
{
	if (a == 0.0)
	{
		if (std::copysign(1.0, a) == 1.0)
		{
			return b;
		}
		return -0.0;
	}
	if (b == 0.0)
	{
		return double_or(b, a);
	}
	if (a < 0.0 && b < 0.0)
	{
		return -double_and(-a, -b);
	}
	if (fabs(a) >= fabs(b))
	{
		return double_or_(a, b);
	}
	return double_or_(b, a);
}

double double_xor(double a, double b)
{
	if (a == 0.0)
	{
		if (std::copysign(1.0, a) == 1.0)
		{
			return b;
		}
		return -b;
	}
	if (b == 0.0)
	{
		return double_xor(b, a);
	}
	if (a < 0.0)
	{
		if (b < 0.0)
		{
			return double_xor(-a, -b);
		}
		return -double_xor(-a, b);
	}
	if (b < 0.0)
	{
		return -double_xor(a, -b);
	}
	if (fabs(a) >= fabs(b))
	{
		return double_xor_(a, b);
	}
	return double_xor_(b, a);
}

void PluginCore::updateLpfCoeffs()
{
	double theta = 2.0 * kPi * lpfCutoff / audioProcDescriptor.sampleRate;
	double d = 1.0 / lpfQ;
	double dSin = (d / 2.0) * std::sin(theta);
	double beta = 0.5 * ((1 - dSin) / (1 + dSin));
	double gamma = (0.5 + beta) * std::cos(theta);
	a_1 = 0.5 + beta - gamma;
	a_0 = a_2 = a_1 / 2.0;
	b_1 = -2.0 * gamma;
	b_2 = 2.0 * beta;
}

void PluginCore::writeBuffer(double input, std::unique_ptr<double[]> &buffer)
{
	buffer[writeIndex] = input;
	writeIndex &= wrapMask;
}

double PluginCore::readBuffer(int delayInSamples, std::unique_ptr<double[]> &buffer)
{
	// --- subtract to make read index
	//     note: -1 here is because we read-before-write,
	//           so the *last* write location is what we use for the calculation
	int readIndex = (writeIndex - 1) - delayInSamples;

	readIndex &= wrapMask;

	return buffer[readIndex];
}

/**
\brief frame-processing method

Operation:
- decode the plugin type - for synth plugins, fill in the rendering code; for FX plugins, delete the if(synth) portion and add your processing code
- note that MIDI events are fired for each sample interval so that MIDI is tightly sunk with audio
- doSampleAccurateParameterUpdates will perform per-sample interval smoothing

\param processFrameInfo structure of information about *frame* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processAudioFrame(ProcessFrameInfo& processFrameInfo)
{
    processFrameInfo.midiEventQueue->fireMidiEvents(processFrameInfo.currentFrame);
	doSampleAccurateParameterUpdates();
	if (getPluginType() == kSynthPlugin ||
		processFrameInfo.numAudioInChannels == 0 ||
		processFrameInfo.numAudioOutChannels == 0)
		return false;

	updateLpfCoeffs();
	bool allFilterMixZero = lpfMix <= 0.0 && hpfMix <= 0.0 && apfMix <= 0.0;
	bool applyFilterPre = !allFilterMixZero && compareEnumToInt(multiFilterPositionEnum::PRE, multiFilterPosition);
	bool applyFilterPost = compareEnumToInt(multiFilterPositionEnum::POST, multiFilterPosition);
	bool flipBoth = compareEnumToInt(phaseFlipEnum::BOTH, phaseFlip);
	double prePhase = (flipBoth || compareEnumToInt(phaseFlipEnum::PRE, phaseFlip)) ? -1.0 : 1.0;
	double postPhase = (flipBoth || compareEnumToInt(phaseFlipEnum::POST, phaseFlip)) ? -1.0 : 1.0;

	double xnL = processFrameInfo.audioInputFrame[0];
	double xnL_lag = xnL;
	if (lagSamples > 0) 
	{
		xnL_lag = readBuffer(lagSamples, buffer_L);
	}

	double xnL_mult = xnL_lag;
	if (applyFilterPre)
	{
		double xnL_lpf = a_0 * xnL_lag + a_1 * x_1.first + a_2 * x_2.first - b_1 * y_1.first - b_2 * y_2.first;
		checkFloatUnderflow(xnL_lpf);
		x_2.first = x_1.first;
		x_1.first = xnL_lag;
		y_2.first = y_1.first;
		y_1.first = xnL_lpf;
		xnL_mult = xnL_lpf * (lpfMix - hpfMix + 2.0 * apfMix) + xnL_lag * (hpfMix - apfMix);
	}

	double xnL_attn = xnL_mult * pow(10.0, modLevel / 20.0) * prePhase;
	double ynL = 0.0;
	
	if (compareEnumToInt(modTypeEnum::XOR, modType))
	{
		ynL = double_xor(xnL, xnL_attn);
	}
	else if (compareEnumToInt(modTypeEnum::AND, modType))
	{
		ynL = double_and(xnL, xnL_attn);
	}
	else if (compareEnumToInt(modTypeEnum::OR, modType))
	{
		ynL = double_or(xnL, xnL_attn);
	}
	if (applyFilterPost)
	{
		double ynL_lpf = a_0 * ynL + a_1 * x_1.first + a_2 * x_2.first - b_1 * y_1.first - b_2 * y_2.first;
		checkFloatUnderflow(ynL_lpf);
		x_2.first = x_1.first;
		x_1.first = ynL;
		y_2.first = y_1.first;
		y_1.first = ynL_lpf;
		ynL = ynL_lpf * (lpfMix - hpfMix + 2.0 * apfMix) + ynL * (hpfMix - apfMix);
	}
	
	ynL = mix * xnL + (1.0 - mix) * ynL * postPhase;

    // --- FX Plugin:
    if (processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFMono)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = ynL;

		writeIndex += 1;
		writeBuffer(xnL, buffer_L);
        return true; /// processed
    }

    // --- Mono-In/Stereo-Out
    if (processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = ynL;
        processFrameInfo.audioOutputFrame[1] = ynL;

		writeIndex += 1;
		writeBuffer(xnL, buffer_L);
        return true; /// processed
    }

	double xnR = processFrameInfo.audioInputFrame[1];
	double xnR_lag = xnR;
	if (lagSamples > 0)
	{
		xnR_lag = readBuffer(lagSamples, buffer_R);
	}

	double xnR_mult = xnR_lag;
	if (applyFilterPre)
	{
		double xnR_lpf = a_0 * xnR_lag + a_1 * x_1.second + a_2 * x_2.second - b_1 * y_1.second - b_2 * y_2.second;
		checkFloatUnderflow(xnR_lpf);
		x_2.second = x_1.second;
		x_1.second = xnR_lag;
		y_2.second = y_1.second;
		y_1.second = xnR_lpf;
		xnR_mult = xnR_lpf * (lpfMix - hpfMix + 2.0 * apfMix) + xnR_lag * (hpfMix - apfMix);
	}

	double xnR_attn = xnR_mult * pow(10.0, modLevel / 20.0) * prePhase;
	double ynR = 0.0;

	if (compareEnumToInt(modTypeEnum::XOR, modType))
	{
		ynR = double_xor(xnR, xnR_attn);
	}
	else if (compareEnumToInt(modTypeEnum::AND, modType))
	{
		ynR = double_and(xnR, xnR_attn);
	}
	else if (compareEnumToInt(modTypeEnum::OR, modType))
	{
		ynR = double_or(xnR, xnR_attn);
	}
	if (applyFilterPost)
	{
		double ynR_lpf = a_0 * ynR + a_1 * x_1.second + a_2 * x_2.second - b_1 * y_1.second - b_2 * y_2.second;
		checkFloatUnderflow(ynR_lpf);
		x_2.second = x_1.second;
		x_1.second = ynR;
		y_2.second = y_1.second;
		y_1.second = ynR_lpf;
		ynR = ynR_lpf * (lpfMix - hpfMix + 2.0 * apfMix) + ynR * (hpfMix - apfMix);
	}

	ynR = mix * xnR + (1.0 - mix) * ynR * postPhase;

    // --- Stereo-In/Stereo-Out
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFStereo &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = ynL;
        processFrameInfo.audioOutputFrame[1] = ynR;

		writeIndex += 1;
		writeBuffer(xnL, buffer_L);
		writeBuffer(xnR, buffer_R);
        return true; /// processed
    }

    return false; /// NOT processed
}

/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- updateOutBoundVariables sends metering data to the GUI meters

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
	// --- update outbound variables; currently this is meter data only, but could be extended
	//     in the future
	updateOutBoundVariables();

    return true;
}

/**
\brief update the PluginParameter's value based on GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- use base class helper
    setPIParamValue(controlID, controlValue);

    // --- do any post-processing
    postUpdatePluginParameter(controlID, controlValue, paramInfo);

    return true; /// handled
}

/**
\brief update the PluginParameter's value based on *normlaized* GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param normalizedValue the new control value in normalized form
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameterNormalized(int32_t controlID, double normalizedValue, ParameterUpdateInfo& paramInfo)
{
	// --- use base class helper, returns actual value
	double controlValue = setPIParamValueNormalized(controlID, normalizedValue, paramInfo.applyTaper);

	// --- do any post-processing
	postUpdatePluginParameter(controlID, controlValue, paramInfo);

	return true; /// handled
}

/**
\brief perform any operations after the plugin parameter has been updated; this is one paradigm for
	   transferring control information into vital plugin variables or member objects. If you use this
	   method you can decode the control ID and then do any cooking that is needed. NOTE: do not
	   overwrite bound variables here - this is ONLY for any extra cooking that is required to convert
	   the GUI data to meaninful coefficients or other specific modifiers.

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postUpdatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- now do any post update cooking; be careful with VST Sample Accurate automation
    //     If enabled, then make sure the cooking functions are short and efficient otherwise disable it
    //     for the Parameter involved
    /*switch(controlID)
    {
        case 0:
        {
            return true;    /// handled
        }

        default:
            return false;   /// not handled
    }*/

    return false;
}

/**
\brief has nothing to do with actual variable or updated variable (binding)

CAUTION:
- DO NOT update underlying variables here - this is only for sending GUI updates or letting you
  know that a parameter was changed; it should not change the state of your plugin.

WARNING:
- THIS IS NOT THE PREFERRED WAY TO LINK OR COMBINE CONTROLS TOGETHER. THE PROPER METHOD IS
  TO USE A CUSTOM SUB-CONTROLLER THAT IS PART OF THE GUI OBJECT AND CODE.
  SEE http://www.willpirkle.com for more information

\param controlID the control ID value of the parameter being updated
\param actualValue the new control value

\return true if operation succeeds, false otherwise
*/
bool PluginCore::guiParameterChanged(int32_t controlID, double actualValue)
{
	/*
	switch (controlID)
	{
		case controlID::<your control here>
		{

			return true; // handled
		}

		default:
			break;
	}*/

	return false; /// not handled
}

/**
\brief For Custom View and Custom Sub-Controller Operations

NOTES:
- this is for advanced users only to implement custom view and custom sub-controllers
- see the SDK for examples of use

\param messageInfo a structure containing information about the incoming message

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMessage(MessageInfo& messageInfo)
{
	// --- decode message
	switch (messageInfo.message)
	{
		// --- add customization appearance here
	case PLUGINGUI_DIDOPEN:
	{
		return false;
	}

	// --- NULL pointers so that we don't accidentally use them
	case PLUGINGUI_WILLCLOSE:
	{
		return false;
	}

	// --- update view; this will only be called if the GUI is actually open
	case PLUGINGUI_TIMERPING:
	{
		return false;
	}

	// --- register the custom view, grab the ICustomView interface
	case PLUGINGUI_REGISTER_CUSTOMVIEW:
	{

		return false;
	}

	case PLUGINGUI_REGISTER_SUBCONTROLLER:
	case PLUGINGUI_QUERY_HASUSERCUSTOM:
	case PLUGINGUI_USER_CUSTOMOPEN:
	case PLUGINGUI_USER_CUSTOMCLOSE:
	case PLUGINGUI_EXTERNAL_SET_NORMVALUE:
	case PLUGINGUI_EXTERNAL_SET_ACTUALVALUE:
	{

		return false;
	}

	default:
		break;
	}

	return false; /// not handled
}


/**
\brief process a MIDI event

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param event a structure containing the MIDI event data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMIDIEvent(midiEvent& event)
{
	return true;
}

/**
\brief (for future use)

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param vectorJoysickData a structure containing joystick data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::setVectorJoystickParameters(const VectorJoystickData& vectorJoysickData)
{
	return true;
}

/**
\brief use this method to add new presets to the list

NOTES:
- see the SDK for examples of use
- for non RackAFX users that have large paramter counts, there is a secret GUI control you
  can enable to write C++ code into text files, one per preset. See the SDK or http://www.willpirkle.com for details

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginPresets()
{
	// **--0xFF7A--**

	// **--0xA7FF--**

    return true;
}

/**
\brief setup the plugin description strings, flags and codes; this is ordinarily done through the ASPiKreator or CMake

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginDescriptors()
{
    pluginDescriptor.pluginName = PluginCore::getPluginName();
    pluginDescriptor.shortPluginName = PluginCore::getShortPluginName();
    pluginDescriptor.vendorName = PluginCore::getVendorName();
    pluginDescriptor.pluginTypeCode = PluginCore::getPluginType();

	// --- describe the plugin attributes; set according to your needs
	pluginDescriptor.hasSidechain = kWantSidechain;
	pluginDescriptor.latencyInSamples = kLatencyInSamples;
	pluginDescriptor.tailTimeInMSec = kTailTimeMsec;
	pluginDescriptor.infiniteTailVST3 = kVSTInfiniteTail;

    // --- AAX
    apiSpecificInfo.aaxManufacturerID = kManufacturerID;
    apiSpecificInfo.aaxProductID = kAAXProductID;
    apiSpecificInfo.aaxBundleID = kAAXBundleID;  /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.aaxEffectID = "aaxDeveloper.";
    apiSpecificInfo.aaxEffectID.append(PluginCore::getPluginName());
    apiSpecificInfo.aaxPluginCategoryCode = kAAXCategory;

    // --- AU
    apiSpecificInfo.auBundleID = kAUBundleID;   /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.auBundleName = kAUBundleName;

    // --- VST3
    apiSpecificInfo.vst3FUID = PluginCore::getVSTFUID(); // OLE string format
    apiSpecificInfo.vst3BundleID = kVST3BundleID;/* MacOS only: this MUST match the bundle identifier in your info.plist file */
	apiSpecificInfo.enableVST3SampleAccurateAutomation = kVSTSAA;
	apiSpecificInfo.vst3SampleAccurateGranularity = kVST3SAAGranularity;

    // --- AU and AAX
    apiSpecificInfo.fourCharCode = PluginCore::getFourCharCode();

    return true;
}

// --- static functions required for VST3/AU only --------------------------------------------- //
const char* PluginCore::getPluginBundleName() { return kAUBundleName; }
const char* PluginCore::getPluginName(){ return kPluginName; }
const char* PluginCore::getShortPluginName(){ return kShortPluginName; }
const char* PluginCore::getVendorName(){ return kVendorName; }
const char* PluginCore::getVendorURL(){ return kVendorURL; }
const char* PluginCore::getVendorEmail(){ return kVendorEmail; }
const char* PluginCore::getAUCocoaViewFactoryName(){ return AU_COCOA_VIEWFACTORY_STRING; }
pluginType PluginCore::getPluginType(){ return kPluginType; }
const char* PluginCore::getVSTFUID(){ return kVSTFUID; }
int32_t PluginCore::getFourCharCode(){ return kFourCharCode; }
