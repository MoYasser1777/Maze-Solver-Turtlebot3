#!/usr/bin/env python3
from wallfollower import WallFollower

SPEED = 0.32
DISTANCE_TO_WALL = 0.4
FOLLOW_SIDE = "left"

def main():

    wall_follower = WallFollower(speed=SPEED, distance_to_wall=DISTANCE_TO_WALL, side=FOLLOW_SIDE)

    distance, timetaken, completed = wall_follower.solve()

    if completed:
        print("Maze Solved!")
    else:
        print("Maze not Solved!")

    print("Total Time :",timetaken,"s\n")
    print("Total Distance :",distance,"m\n")

if __name__ == '__main__':
    main()
