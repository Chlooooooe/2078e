class Point {
    public:
        
        double x, y, orientation;
        
        Point(double x_position, double y_position, double degrees) {
            x, y, degrees = x_position, y_position, orientation;
        }
};