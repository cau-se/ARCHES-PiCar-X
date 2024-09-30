---
title: Automated Testing
has_children: false
nav_order: 5
parent: Explore

---

<link rel="stylesheet" href="{{ site.baseurl }}{% link assets/css/tabs.css %}">
<script src="{{ site.baseurl }}{% link assets/js/tabs.js %}"> </script>

# Testing the ARCHES PiCar-X
Test automation has been identified as one of the most prominent areas in the testing of embedded software. However, achieving effective automated quality assurance remains challenging, mainly due to the need for hardware involvement in the testing process. This form Hardware-in-the-Loop testing often requires a continuous connection to the testing environment, which can be expensive and impractical, especially for small and medium-sized enterprises. To address this issue, the digital twin prototype is designed to operate independently of the physical system while still enabling the testing of real embedded software.

# Continuous Integration / Continuous Delivery with GitHub Actions
Each time a user commits changes, a GitHub Runner is triggered to build a Docker container. All dependencies are loaded into this container, and the code is compiled. This build step could also include static software checks to further assess the quality of the code. Once the containers are successfully built, unit tests are executed on the module under test. If these tests pass, we proceed to the next critical phase: integration testing. To illustrate how the digital twin prototype can be used for automated integration testing without requiring the real hardware, we have create an integration test from the script used for speed measurement. This integration test ensures that the digital model in the simulation runs at the same speed as the real one. After successful integration tests, the different Docker containers are released. The CI/CD pipelines are executed on three runners in parallel, one for x64 system, one runner on a RaspberryPi 3 (arm32v7), and the third on a RaspberryPi 4 (arm64v8). Notice that only the X64 runner can execute the integratin tests that use virtual context from the [GAZEBO](https://gazebosim.org/) simulation. GAZEBO has no ARM build available. The results of the different builds can be seen on the [GitHub Actions overview page](https://github.com/cau-se/ARCHES-PiCar-X/actions).

![CI/CD Pipeline for the PiCar-X.](./assets/images/picarx-cicd.jpg)

# Manual Integration Testing
Since the CI/CD pipelines also only use Docker for test execution, the whole workflow can be executed manually. The different workflows implemented in this project are shown in the [.github folder](https://github.com/cau-se/ARCHES-PiCar-X/tree/main/.github).


<div class="tab-container" id="manualtesting">
  <ul class="tab-list">
<li class="tab active" data-tab="tab1-1">The full DTP on an X64 machine</li>
<li class="tab" data-tab="tab1-2">On a RaspberryPi 3 (arm32v7)</li>
<li class="tab" data-tab="tab1-2">On a RaspberryPi 4 (arm64v8)</li>
  </ul>
  <div class="tab-content active" id="tab1-1">
  {% highlight console %}
# Build the core container
TAG=latest docker compose -f docker-compose-core.yml build --no-cache

# Build the DTP, in this case without Gazebo GUI (headless mode)
TAG=latest docker compose -f docker-compose-dtp-headless-gazebo.yml build --no-cache

# Execute Unit Tests in the core container
docker run --rm --name picarx-unittest ghcr.io/cau-se/arches-picar-x/picarx:latest pytest ./src/core/picarx/tests

# Run integration tests for the dc motor
docker run --rm --name dcmotor-integration-test -v /sys/class/gpio:/sys/class/gpio -v /dev/i2c-0:/dev/i2c-0 --privileged  ghcr.io/cau-se/arches-picar-x/drivers/dcmotor:latest rostest picarx_dcmotor_driver integration_tests.test i2c_port:=/dev/i2c-0

# Start the DTP setup using GAZEBO headless
i2c=/dev/i2c-0 docker compose -f docker-compose-dtp-headless-gazebo.yml up -d

# Execute the integration test for speed measurement
docker exec picar-x-picarx-gazebo-control-1 /bin/bash -c "source ./install/setup.bash && sleep 30; python3 ./src/simulation/picarx_control/tests/steering_integration_test.py"

# Turn off the DTP
docker compose -f docker-compose-dtp-headless-gazebo.yml down {% endhighlight %}  
  </div>
  <div class="tab-content" id="tab1-2">
  {% highlight console %}
# Build the core container
TAG=latest-arm32v7 ARCH=arm32v7 docker compose -f docker-compose-core.yml build --no-cache

# Build the DTP, in this case without Gazebo
TAG=latest-arm32v7 ARCH=arm32v7 docker compose -f docker-compose-dtp-no-gazebo.yml build --no-cache

# Execute Unit Tests in the core container
docker run --rm --name picarx-unittest ghcr.io/cau-se/arches-picar-x/picarx:latest-arm32v7 pytest ./src/core/picarx/tests

# Run integration tests for the dc motor
docker run --rm --name dcmotor-integration-test -v /sys/class/gpio:/sys/class/gpio -v /dev/i2c-11:/dev/i2c-11 --privileged  ghcr.io/cau-se/arches-picar-x/drivers/dcmotor:latest-arm32v7 rostest picarx_dcmotor_driver integration_tests.test i2c_port:=/dev/i2c-11 {% endhighlight %}  
  </div>
  <div class="tab-content" id="tab1-3">
  {% highlight console %}
# Build the core container
TAG=latest-arm64v8 ARCH=arm64v8 docker compose -f docker-compose-core.yml build --no-cache

# Build the DTP, in this case without Gazebo
TAG=latest-arm64v8 ARCH=arm64v8 docker compose -f docker-compose-dtp-no-gazebo.yml build --no-cache

# Execute Unit Tests in the core container
docker run --rm --name picarx-unittest ghcr.io/cau-se/arches-picar-x/picarx:latest-arm64v8 pytest ./src/core/picarx/tests

# Run integration tests for the dc motor
docker run --rm --name dcmotor-integration-test -v /sys/class/gpio:/sys/class/gpio -v /dev/i2c-11:/dev/i2c-11 --privileged  ghcr.io/cau-se/arches-picar-x/drivers/dcmotor:latest-arm64v8 rostest picarx_dcmotor_driver integration_tests.test i2c_port:=/dev/i2c-11 {% endhighlight %}  
  </div>

</div>