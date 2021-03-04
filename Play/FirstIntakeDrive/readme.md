# 2021-03-03 Test Run

##  Investigated left intake does not move or move in weird way (fixed)

The fix is to make sure calling vex::task::sleep() at the end of loop.

## Investigated the issue that left wheel do not rotate. (SEVERE problem, NOT resolved)

- added logging using ScrollingScreen to verify if left, right motor speeds are correct: no issue found
- tested front wheels and back wheels separately: front is fine, back left is stuck

Conclusion: Wheel and gear are in contact, the problem is most severe in back left. Why is it? We did not find the desired gear when building the base, so a bigger wheel was chosen.

Now it is extremely hard to add extra width to the drive train band as it will make all vertical structure to add width.

LESSON LEARNED: <red> we MUST test drive train when it is built. Do not wait until the robot is fully assembled! </red>
