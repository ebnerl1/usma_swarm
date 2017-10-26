/// ---------------------------------------------------------------------------
/// @section LICENSE
///  
/// Copyright (c) 2016 Georgia Tech Research Institute (GTRI) 
///               All Rights Reserved
///  
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
///  
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ---------------------------------------------------------------------------
/// @file filename.ext
/// @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu> 
/// @author Eric Squires <eric.squires@gtri.gatech.edu>
/// @version 1.0
/// ---------------------------------------------------------------------------
/// @brief A brief description.
/// 
/// @section DESCRIPTION
/// A long description.
/// ---------------------------------------------------------------------------
#include <iostream>

#include <scrimmage/common/Utilities.h>

#include <scrimmage/plugin_manager/PluginManager.h>

#include "AutonomySample.h"

using std::cout;
using std::endl;

REGISTER_PLUGIN(scrimmage::Autonomy, AutonomySample, AutonomySample_plugin)

AutonomySample::AutonomySample()
{
}

void AutonomySample::init(std::map<std::string,std::string> &params)
{    
    desired_state_.vel() = scrimmage::get_double("speed", params, 0)*Eigen::Vector3d::UnitX();
    desired_state_.quat().set(0,0,state_.quat().yaw());
    desired_state_.pos() = state_.pos()(2)*Eigen::Vector3d::UnitZ();
}

bool AutonomySample::step_autonomy(double t, double dt)
{            
    return true;
}
