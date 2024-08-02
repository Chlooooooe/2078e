extern bool armIntake;
extern int count;
extern bool detected;

void bothMove(int vel);
void Delay(int time);
std::tuple<bool, bool, int> Delay(int time, bool armIntake, bool detected, int count);
void scuffed();
void mogoRush();
void ringRushRed();
void soloWPRed();
void soloWPBlue();
void ringRushBlue();