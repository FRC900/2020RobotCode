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

                void pause(bool paused);
                bool getPause() const;
                bool pauseChanged();
                void resetPause();
                
                void play(bool playing);
                bool getPlay() const;
                bool playChanged();
                void resetPlay();

                void stop(bool stop);
                bool getStop() const;
                bool stopChanged();
                void resetStop(bool &stop);

                void loadMusic(std::string file_path);
                bool getLoadMusic() const;
                bool loadMusicChanged(std::string file_path);
                void resetLoadMusic();

                void addInstruments(std::vector<std::string> instruments);
                bool getAddInstruments() const;
                bool instrumentsChanged(std::vector<std::string> &instruments);
                void resetAddInstruments();

                void setClearInstruments(bool clear_instruments);
                bool getClearInstruments() const;
                bool clearInstrumentsChanged(bool &clear_instruments);
                void resetClearInstruments();

	private:
		std::vector<std::string> instruments_;

                bool pause_changed_;
                bool play_changed_;
                bool stop_changed_;

                bool load_music_;
                bool load_music_changed_;
                bool add_instruments_;
                bool add_instruments_changed_;
                bool clear_instruments_;
                bool clear_instruments_changed_;
};

// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef CommandHandle<const OrchestraCommand> OrchestraCommandHandle;
typedef CommandHandle<OrchestraCommand> OrchestraWritableCommandHandle;
class OrchestraCommandInterface : public HardwareResourceManager<OrchestraCommandHandle> {};
}
