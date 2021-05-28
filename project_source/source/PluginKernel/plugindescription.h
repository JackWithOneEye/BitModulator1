// --- CMAKE generated variables for your plugin

#include "pluginstructures.h"

#ifndef _plugindescription_h
#define _plugindescription_h

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)
#define AU_COCOA_VIEWFACTORY_STRING STR(AU_COCOA_VIEWFACTORY_NAME)
#define AU_COCOA_VIEW_STRING STR(AU_COCOA_VIEW_NAME)

// --- AU Plugin Cocoa View Names (flat namespace) 
#define AU_COCOA_VIEWFACTORY_NAME AUCocoaViewFactory_57360ECC9B163ED1BFD06DA2F790BB58
#define AU_COCOA_VIEW_NAME AUCocoaView_57360ECC9B163ED1BFD06DA2F790BB58

// --- BUNDLE IDs (MacOS Only) 
const char* kAAXBundleID = "developer.aax.bitmodulator1.bundleID";
const char* kAUBundleID = "developer.au.bitmodulator1.bundleID";
const char* kVST3BundleID = "developer.vst3.bitmodulator1.bundleID";

// --- Plugin Names 
const char* kPluginName = "BitModulator1";
const char* kShortPluginName = "BitModulator1";
const char* kAUBundleName = "BitModulator1_AU";

// --- Plugin Type 
const pluginType kPluginType = pluginType::kFXPlugin;

// --- VST3 UUID 
const char* kVSTFUID = "{57360ecc-9b16-3ed1-bfd0-6da2f790bb58}";

// --- 4-char codes 
const int32_t kFourCharCode = 'BMM1';
const int32_t kAAXProductID = 'BMM1';
const int32_t kManufacturerID = 'ASPK';

// --- Vendor information 
const char* kVendorName = "My Company";
const char* kVendorURL = "www.myplugins.com";
const char* kVendorEmail = "support@myplugins.com";

// --- Plugin Options 
const bool kWantSidechain = false;
const uint32_t kLatencyInSamples = 0;
const double kTailTimeMsec = 0.000000;
const bool kVSTInfiniteTail = false;
const bool kVSTSAA = false;
const uint32_t kVST3SAAGranularity = 1;
const uint32_t kAAXCategory = 0;

#endif
