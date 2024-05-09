class Pose{
    public:
        float x;
        float y;
        float heading; //radians

        Pose(float x, float y, float heading){
            this->x = x;
            this->y = y;
            this->heading = heading;
        }
};