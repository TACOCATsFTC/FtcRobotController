# Project Preferences

- Use Java and the existing FTC SDK patterns in `TeamCode`.
- Reuse `Yoshi` and `RobotConstants` for current-season hardware and settings.
- Keep hardware access inside robot hardware classes rather than directly in OpModes.
- Give autonomous movements a timeout, stop motors after each movement, and honor stop requests.
- Use telemetry for driver-station status and Android logging for detailed diagnostics.
- Preserve the current OTOS coordinate convention: +X right, +Y forward, and positive heading counterclockwise.
- Keep autonomous route waypoints near the top of `runOpMode()` so they are easy to tune.
- Validate TeamCode changes with `.\gradlew.bat :TeamCode:compileDebugJavaWithJavac`.
