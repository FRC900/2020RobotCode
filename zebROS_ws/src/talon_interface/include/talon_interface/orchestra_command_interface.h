#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{
class OrchestraCommand
{
	public:
		OrchestraCommand();
		~OrchestraCommand();

                void pause();
                bool getPause() const;
                bool pauseChanged();
                void resetPause();
                
                void play();
                bool getPlay() const;
                bool playChanged();
                void resetPlay();

                void stop();
                bool getStop() const;
                bool stopChanged();
                void resetStop(bool &stop);

                void loadMusic(std::string file_path);
                bool getMusic() const;
                bool musicChanged(std::string &file_path);
                void resetMusic();

                void addInstruments(std::vector<std::string> instruments);
                bool getInstruments() const;
                bool instrumentsChanged(std::vector<std::string> &instruments);
                void resetInstruments();

                void clearInstruments();
                bool getClearInstruments() const;
                bool clearInstrumentsChanged();
                void resetClearInstruments();

	private:
                bool pause_changed_;
                bool play_changed_;
                bool stop_changed_;

                std::string file_path_;
                bool load_music_changed_;
		std::vector<std::string> instruments_;
                bool add_instruments_changed_;
                bool instruments_cleared_;
                bool clear_instruments_changed_;
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef CommandHandle<const OrchestraCommand> OrchestraCommandHandle;
typedef CommandHandle<OrchestraCommand> OrchestraWritableCommandHandle;
class OrchestraCommandInterface : public HardwareResourceManager<OrchestraCommandHandle> {};
}
