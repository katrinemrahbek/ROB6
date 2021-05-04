/****************************************************************************\
 * Copyright (C) 2018 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

#include <royale/String.hpp>

namespace royale
{
    /*!
     *  Royale allows to set different filter levels. Internally these represent
     *  different configurations of the processing pipeline.
     */
    enum class FilterLevel
    {
        Off = 0,        ///< Turn off all filtering of the data (validation will still be enabled)
        Dynamic = 1,
        Static = 2,
        DynamicBinning = 3,
        StaticBinning = 4,
        IR1 = 5,
        IR2 = 6,
        AF1 = 7,
        CM1 = 8,
        Binning_1_Basic = 9,
        Binning_2_Basic = 10,
        Binning_3_Basic = 11,
        Binning_4_Basic = 12,
        Binning_8_Basic = 13,
        Binning_10_Basic = 14,
        Binning_1_Efficiency = 15,
        Binning_2_Efficiency = 16,
        Binning_3_Efficiency = 17,
        Binning_4_Efficiency = 18,
        Binning_8_Efficiency = 19,
        Binning_10_Efficiency = 20,
        Legacy = 200,   ///< Standard settings for older cameras
        Full = 255,     ///< Enable all filters that are available for this camera
        Custom = 256    ///< Value returned by getFilterLevel if the processing parameters differ from all of the presets
    };

    ROYALE_API royale::String getFilterLevelName (royale::FilterLevel level);

    ROYALE_API royale::FilterLevel getFilterLevelFromName (royale::String name);
}
