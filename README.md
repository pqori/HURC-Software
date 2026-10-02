### Development Setup
When you first clone the repo, you need to build the base docker image. This will install ROS, the ZED SDK, the Teensy build system, and all of the current package dependencies:
```bash
make image
```
This command takes a while, so you should only re-run it whenever new dependencies or setup commands are added to the project that would be annoying to re-run often. You can also run `make update` from inside the container to install/update any ros package dependencies without rebuilding the whole image (but these will not persist across creating new containers).

Once the image is built, you can run the following command to open a shell in the development environment:
```bash
make ros
```
Any changes made to the container's filesystem will be saved until you run either `make stop` or `make restart`.

Once the container is running, you can access any GUI apps by opening http://localhost:8080/vnc.html in a browser.

### Building
Inside the container or on the jetson, you can use the following commands to build and use ros packages:
```bash
make build # or build-package-name
source install/local_setup.bash
```
