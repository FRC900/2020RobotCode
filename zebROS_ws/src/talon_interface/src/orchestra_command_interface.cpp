#include "talon_interface/orchestra_command_interface.h"

namespace hardware_interface
{

// Set up default values
// Set most of the changed_ vars to true
// to force a write of these values to the Talon
// That should put the talon in a known state
// rather than relying on them being setup to
// a certain state previously
OrchestraCommand::OrchestraCommand() :
	pause_changed_(true),
	play_changed_(true),
	stop_changed_(true),
        file_path_(""),
	load_music_changed_(false),
        instruments_{},
        add_instruments_changed_(false),
        clear_instruments_changed_(true)
{
}

OrchestraCommand::~OrchestraCommand()
{
}

void OrchestraCommand::pause()
{
	pause_changed_ = true;
}
bool OrchestraCommand::getPause(void) const
{
	return pause_changed_; //TODO this is wrong
}
bool OrchestraCommand::pauseChanged()
{
    if(pause_changed_)
    {
        pause_changed_ = false;
        return true;
    }
    return false;
}
void resetPause()
{
    pause_changed_ = true;
}

void OrchestraCommand::play()
{
	play_changed_ = true;
}
bool OrchestraCommand::getPlay(void) const
{
	return play_changed_; //TODO this is wrong
}
bool OrchestraCommand::playChanged()
{
    if(play_changed_)
    {
        play_changed_ = false;
        return true;
    }
    return false;
}
void resetPlay()
{
    play_changed_ = true;
}

void OrchestraCommand::stop()
{
	stop_changed_ = true;
}
bool OrchestraCommand::getStop(void) const
{
	return stop_changed_; //TODO this is wrong
}
bool OrchestraCommand::stopChanged()
{
    if(stop_changed_)
    {
        stop_changed_ = false;
        return true;
    }
    return false;
}
void resetStop()
{
    stop_changed_ = true;
}

void OrchestraCommand::loadMusic(std::string file_path)
{
    file_path_ = file_path;
    load_music_changed_ = true;
}
std::string OrchestraCommand::getMusic() const
{
    return file_path_;
}
OrchestraCommand::musicChanged(std::string &file_path)
{
    if(load_music_changed_)
    {
        load_music_changed_ = false;
        file_path = file_path_;
        return true;
    }
    return false;
}
OrchestraCommand::resetMusic()
{
    load_music_changed_ = true;
}

OrchestraCommand::addInstruments(std::vector<std::string> instruments)
{
    add_instruments_changed_ = true;
    instruments_ = instruments;
}
OrchestraCommand::getInstruments() const
{
    return instruments_;
}
bool OrchestraCommand::instrumentsChanged(std::vector<std::string> &instruments)
{
    if(add_instruments_changed_)
    {
        add_instruments_changed_ = false;
        instruments = instruments_;
        return true;
    }
    return false;
}
void OrchestraCommand::resetInstruments()
{
    instruments_changed_ = true;
}

void OrchestraCommand::clearInstruments()
{
    clear_instruments_changed_ = true;
}
bool getInstrumentsCleared()
{
    return true; // TODO Wht is this function supposed to do?
}
bool clearInstrumentsChanged()
{
    if(clear_instruments_changed_)
    {
        clear_instruments_changed_ = false;
        return true;
    }
    return false;
}
void resetClearInstruments()
{
    clear_instruments_changed_ = true;
}

} // namespace
