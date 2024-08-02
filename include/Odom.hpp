
class Odom {
    public:
        Odom(double absolute_x, double absolute_y, double start_angle);
        void update_offsets();
        void update_previous();
        double getX();
        double getY();

};