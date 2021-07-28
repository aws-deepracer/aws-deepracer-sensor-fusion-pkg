///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <memory>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <fcntl.h>
#include <unistd.h>

namespace SensorFusion {
        
    /// Check's if the desired file exist
    /// @returns True if file exist, false otherwise
    /// @param filePath Full path to desired file for which to validate existence
    bool checkFile(const std::string &filePath);

    /// Writes the given sensor configuration map to file
    /// @param jsonValue json value to write to file
    /// @param filePath Full path to the desired file for which to write the given json.
    void writeJSONToFile(Json::Value jsonValue, const std::string &filePath);

    /// Return evenly spaced numbers between the specified interval
    /// @param start Starting value of the sequence
    /// @param end Ending value of the sequence
    /// @param num Number of samples to generate
    std::vector<float> linspace(float start, float end, float num);

    /// Create a one dimensional interpolation for monotonically increasing values
    /// @param sortedX The values at which the interpolation has to be evaluated
    /// @param sortedXp X-coordinates of the data points (must be increasing)
    /// @param sortedFp Y-coordinates of the data points (same length as X)
    std::vector<float> interp(const std::vector<float>& sortedX, const std::vector<float>& sortedXp, const std::vector<float>& sortedFp);
}
#endif