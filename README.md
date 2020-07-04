# youbot_ad_sim
This project includes many subrepos to provide a playground used for autonomous driving using the KUKA youbot.

I made such repos submodules which I expected not to touch. Other repos has been forked and made subtree.

## License
This project is licensed with the GNU V3 License

## Preparation
1. clone this package
```bash
git clone https://github.com/Unoriginell/youbot_ad_sim.git
```
2. get submodules
```bash
git submodule update --init --recursive
```

3. build package (CI/CD not implemeted yet)
```bash
catkin_make
```
4. source package
```bash
source devel/setup.bash
```

## Let it run
#### Run the slam algorithm with the keyboard control (focus in terminal required)
```bash
roslaunch youbot_ad_sim slam_control.launch
```

#### Save the created map
```bash
rosrun map_server map_saver -f "NAME_OF_MAP"
```

#### Run Navigation (coming soon...)

