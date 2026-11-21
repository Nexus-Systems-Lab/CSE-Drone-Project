# Video guide and transcript

## Demonstration

[Watch or download the 44-second technical voice guide](https://github.com/Uvin-Kaveesh/CSE-Drone-Project/releases/download/demo-2026/drone_project_technical_voice_guide.mp4) · [Download subtitles](../media/captions.srt)

This video uses the team's actual 2026 prototype footage. The original background-song audio track was removed in full. The only audio in the published cut is an English voice guide synthesized with HeyGen Voice. Captions were timed from the generated narration's word timestamps. The picture remains in its source order at normal speed; the export is 720 × 1280 pixels, 60 fps, H.264/AAC MP4.

| Approximate source time | Visible material | Explanation |
| --- | --- | --- |
| 0–4 s | Controller and laptop | Introduces the ESP32 quadcopter prototype |
| 4–11 s | OLED close-up | Status display and MPU6050 sensor role |
| 12–20 s | Motor and dashboard views | Roll/pitch feedback and four ESC commands |
| 20–31 s | Wi-Fi controls and response plots | Motor adjustment, plots, and reported gains |
| 31–36 s | Powered prototype on the workbench | Hardware demonstration and nominal update interval |
| 36–44 s | Original title cards | Scope and validation caveat |

## Spoken transcript

> This is our ESP32 quadcopter control prototype. The OLED displays its state, while an MPU6050 measures acceleration and angular rate. The controller estimates roll and pitch, compares both with zero degrees, then mixes corrections into four ESC commands. Here, the dashboard adjusts motor output and plots the response. The reported gains are P: one point five, I: zero, and D: zero point zero five, so this is P D control. The loop is scheduled every ten milliseconds. The motor test demonstrates the prototype hardware; stable hover and automatic altitude hold still require separate validation.

## Evidence boundary

The source video's end card is titled **“Drone Hover Test.”** The visible test footage shows the powered prototype and interface but does not, by itself, establish quantified stable free-flight hover. The group report describes the roll/pitch controller and normalized Simulink results, while Section 6 says automatic height control is not implemented. The voice guide states these limits so the video is not presented as proof of an unmeasured result.

The prototype video and images are original project material. The poster image is a designed cover made from a frame of the same source video, not an AI-generated drone image.
