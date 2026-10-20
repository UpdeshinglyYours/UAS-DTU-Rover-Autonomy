# Everything the recorder's live status line shows, as fields, plus the
# line itself. Published at status_interval (default 4 Hz).
# Unknown floats are NaN; unknown integers are -1.

builtin_interfaces/Time stamp
string text                 # the full status line, as fcb_record.py printed it

# -- video -------------------------------------------------------------
bool video_open             # a capture is open (it may still be sending nothing)
float32 fps                 # measured, after the board's repeats are dropped
uint64 frames_seen
uint64 frames_skipped       # newer frames overtook the consumer
uint64 repeats_dropped      # Oppila board duplicates thrown away
bool picture_blank          # every pixel one colour: board gets no image over LVDS
string video_device

# -- recording ---------------------------------------------------------
bool recording
bool recording_has_video    # false while recording telemetry only
bool started_by_hand        # start_recording / autostart, not the RC switch
bool write_failed           # a write failed (full disk?); nothing starts until restart
string recording_base       # e.g. /home/uas/flight_recordings/fcb-20260903-114530
uint32 segment
uint64 segment_frames
float32 recording_elapsed_s
float32 disk_free_gb

# -- camera control (VISCA) --------------------------------------------
bool control_ok
string control_port
int32 control_baud
float32 zoom_ratio
int32 zoom_position         # raw VISCA counts
float32 zoom_target_ratio
string zoom_source          # "rc", "manual", or another commander name
string imaging_mode         # RGB, IR, RGB+IR, AUTO, or "" if unknown
uint32 snapshots

# -- flight controller -------------------------------------------------
string fc_source            # pymavlink, mavros, or none
bool fc_present             # a source is attached
bool fc_connected
float32 rc_age_s
