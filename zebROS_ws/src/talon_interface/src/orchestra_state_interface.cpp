#include "talon_interface/orchestra_state_interface.h"

namespace hardware_interface
{

// Set up default values
// Set most of the changed_ vars to true
// to force a write of these values to the Talon
// That should put the talon in a known state
// rather than relying on them being setup to
// a certain state previously
OrchestraState::OrchestraState(int orchestra_id) :
        orchestra_id_(orchestra_id),
        instruments_{},
        chirp_file_path_(""),
        is_playing_(false),
        is_paused_(false),
        is_stopped_(true)
{
}

OrchestraState::~OrchestraState()
{
}

void OrchestraState::setChirpFilePath(std::string chirp_file_path)
{
    chirp_file_path_ = chirp_file_path;
}
std::string OrchestraState::getChirpFilePath() const
{
    return chirp_file_path_;
}

void OrchestraState::setInstruments(std::vector<std::string> instruments)
{
    instruments_ = instruments;
}
std::vector<std::string> OrchestraState::getInstruments() const
{
    return instruments_;
}

void OrchestraState::setIsPaused(bool is_paused)
{
    is_paused_ = is_paused;
}
bool OrchestraState::getIsPaused() const
{
    return is_paused_;
}

void OrchestraState::setIsPlaying(bool is_playing)
{
    is_playing_ = is_playing;
}
bool OrchestraState::getIsPlaying() const
{
    return is_playing_;
}

void OrchestraState::setIsStopped(bool is_stopped)
{
    is_stopped_ = is_stopped;
}
bool OrchestraState::getIsStopped() const
{
    return is_stopped_;
}
}
