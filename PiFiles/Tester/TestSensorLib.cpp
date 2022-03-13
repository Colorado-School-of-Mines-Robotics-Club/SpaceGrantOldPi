//
// Created by mmurr on 2/18/2022.
//

#include "TestSensorLib.h"

#include <cmath>
#include <utility>

#define CONFIG_NAME "../Input.json"

void setBotPosition(float x, float y);

Sensor* SENSOR_NAME;

Wheel* WHEEL1;
Wheel* WHEEL2;
Wheel* WHEEL3;
Wheel* WHEEL4;

bool drawing = true;
double lastTime;

using json = nlohmann::json;

#define BODY_WIDTH .16
#define WHEEL_DIAMETER 0.16
#define WHEEL_OFFSET 0.162

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720

#define INITIAL_WIDTH_DISTANCE 10.0f // meters

#define SPEED .25 // Wheel rotations/sec
#define TURN_SPEED .05 // Rotations/s

#define STOPPED 0
#define FORWARD 1
#define BACKWARD -1

#define PUSH_SENSOR_HEIGHT 0.02

#define BEACON_DIAMETER 0.2
#define BEACON_LENGTH 15
#define BEACON_ROTATION_RATE 1.6 //Rotations per sec

#define ZOOM_AMOUNT 0.1

#define dist(A, B, N) A - B > N - (A - B - 1) ? std::min(A - B, N - (A - B - 1)) : -std::min(A - B, N - (A - B - 1))

struct{
    sf::RectangleShape* shape;
    sf::Vector2<float> position;
    float rotation;
} botBody;

struct wheel{
    sf::CircleShape* shape;
    sf::Vector2<float> position;
    sf::VertexArray directionLine;
    Wheel* wheel;
};

struct CircleObstacle{
    sf::CircleShape* shape;
    sf::Vector2<float> position;
    float radius;
    float height;
};

struct RectObstacle{
    sf::RectangleShape* shape;
    sf::Vector2<float> position;
    sf::Vector2<float> size;
    float rotation;
    float height;
};

struct{
    sf::CircleShape* shape;
    sf::Vector2<float> position;
    float rotation;
    sf::VertexArray directionLine;
}beacon;

wheel wheels[4];

std::vector<CircleObstacle> circleObstacles;
std::vector<RectObstacle> rectObstacles;

bool cameraLocked = false;

sf::VertexArray scannerLine(sf::LinesStrip, 2);

sf::Vector2<float> cameraPosition(0, 0);
float cameraWidth = INITIAL_WIDTH_DISTANCE;

bool inclined = false;

double dot(sf::Vector2<double> A, sf::Vector2<double> B){
    return A.x*B.x + A.y*B.y;
}

Vector3 cross(Vector3 A, Vector3 B){
    return Vector3({
        .x = A.y*B.z - A.z*B.y,
        .y = A.z*B.x - A.x*B.z,
        .z = A.x*B.y - A.y*B.x
    });
}

float getHeight(sf::Vector2<float> position){
    float output = 0;

    for(auto & obstacle : circleObstacles){
        double distance = std::sqrt(std::pow((obstacle.position.x - position.x), 2) + std::pow((obstacle.position.y - position.y), 2));
        if(distance > obstacle.radius) continue;

        output += obstacle.height/2*std::cos(M_PI/obstacle.radius*distance) + obstacle.height/2;
    }

    for(auto & obstacle : rectObstacles){
        sf::Vector2<double> bPos = {obstacle.size.x/2*std::cos(obstacle.rotation*M_PI/180) - obstacle.size.y/2*std::sin(obstacle.rotation*M_PI/180) + obstacle.position.x,
                                   obstacle.size.y/2*std::cos(obstacle.rotation*M_PI/180) + obstacle.size.x/2*std::sin(obstacle.rotation*M_PI/180) + obstacle.position.y};
        sf::Vector2<double> dPos = {2*obstacle.position.x - bPos.x,
                                    2*obstacle.position.y - bPos.y};
        sf::Vector2<double> aPos = {-obstacle.size.x/2*std::cos(obstacle.rotation*M_PI/180) - obstacle.size.y/2*std::sin(obstacle.rotation*M_PI/180) + obstacle.position.x,
                                    obstacle.size.y/2*std::cos(obstacle.rotation*M_PI/180) - obstacle.size.x/2*std::sin(obstacle.rotation*M_PI/180) + obstacle.position.y};

        if(0 < dot({position.x - aPos.x, position.y - aPos.y}, {bPos.x - aPos.x, bPos.y - aPos.y}) &&
            dot({position.x - aPos.x, position.y - aPos.y}, {bPos.x - aPos.x, bPos.y - aPos.y}) < dot({bPos.x - aPos.x, bPos.y - aPos.y}, {bPos.x - aPos.x, bPos.y - aPos.y}) &&
            0 < dot({position.x - aPos.x, position.y - aPos.y}, {dPos.x - aPos.x, dPos.y - aPos.y}) &&
            dot({position.x - aPos.x, position.y - aPos.y}, {dPos.x - aPos.x, dPos.y - aPos.y}) < dot({dPos.x - aPos.x, dPos.y - aPos.y}, {dPos.x - aPos.x, dPos.y - aPos.y})){

            output += obstacle.height;
        }
    }
    return output;
}

void update(){
    double deltaTime = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count() - lastTime;
    lastTime = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();

    sf::Vector2<float> avgDir(0, 0);
    for(auto & wheelMove : wheels){
        while(wheelMove.wheel->rotation > 180){
            wheelMove.wheel->rotation -= 360;
        }
        while(wheelMove.wheel->rotation < -180){
            wheelMove.wheel->rotation += 360;
        }

        wheelMove.wheel->rotation += wheelMove.wheel->turning*TURN_SPEED*360*deltaTime/1000;

        avgDir.x += wheelMove.wheel->running*cos((wheelMove.wheel->rotation + botBody.rotation)*M_PI/180);
        avgDir.y += wheelMove.wheel->running*sin((wheelMove.wheel->rotation + botBody.rotation)*M_PI/180);

        wheelMove.wheel->position += wheelMove.wheel->running*SPEED*deltaTime/1000;
    }
    avgDir.x /= 4;
    avgDir.y /= 4;

    float avgTorque = 0;
    avgTorque -= wheels[0].wheel->running*WHEEL_OFFSET*cos(wheels[0].wheel->rotation*M_PI/180);
    avgTorque += wheels[1].wheel->running*WHEEL_OFFSET*sin(wheels[1].wheel->rotation*M_PI/180);
    avgTorque += wheels[2].wheel->running*WHEEL_OFFSET*cos(wheels[2].wheel->rotation*M_PI/180);
    avgTorque -= wheels[3].wheel->running*WHEEL_OFFSET*sin(wheels[3].wheel->rotation*M_PI/180);

    avgTorque /= 4;

    botBody.rotation += avgTorque*360/(2*WHEEL_OFFSET)*WHEEL_DIAMETER*SPEED*deltaTime/1000;
    setBotPosition(botBody.position.x + avgDir.x*WHEEL_DIAMETER*M_PI*SPEED*deltaTime/1000, botBody.position.y + avgDir.y*WHEEL_DIAMETER*M_PI*SPEED*deltaTime/1000);

    Vector3 gyro = sensor->getGyro();
    Vector3 plane = cross({static_cast<float>(std::cos(-gyro.y*M_PI/180)), 0, static_cast<float>(std::sin(-gyro.y*M_PI/180))}, {0, static_cast<float>(std::cos(gyro.x*M_PI/180)), static_cast<float>(std::sin(gyro.x*M_PI/180))});
    double angle = std::acos(plane.z)*180/M_PI;
    if(abs(angle) > sensor->gyroAlertAngle && !inclined){
        sensor->gyroAlertCallback(gyro);
        inclined = true;
    }else if(abs(angle) < sensor->gyroAlertAngle && inclined){
        inclined = false;
    }

    if(!wheels[0].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180))}) > PUSH_SENSOR_HEIGHT){
        wheels[0].wheel->pressurePressed = true;
        wheels[0].wheel->pressureAlertCallback(0);
    }else if(wheels[0].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180))}) < PUSH_SENSOR_HEIGHT){
        wheels[0].wheel->pressurePressed = false;
    }

    if(!wheels[1].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180))}) > PUSH_SENSOR_HEIGHT){
        wheels[1].wheel->pressurePressed = true;
        wheels[1].wheel->pressureAlertCallback(1);
    }else if(wheels[1].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180))}) < PUSH_SENSOR_HEIGHT){
        wheels[1].wheel->pressurePressed = false;
    }

    if(!wheels[2].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180))}) > PUSH_SENSOR_HEIGHT){
        wheels[2].wheel->pressurePressed = true;
        wheels[2].wheel->pressureAlertCallback(2);
    }else if(wheels[2].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x + (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180))}) < PUSH_SENSOR_HEIGHT){
        wheels[2].wheel->pressurePressed = false;
    }

    if(!wheels[3].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180))}) > PUSH_SENSOR_HEIGHT){
        wheels[3].wheel->pressurePressed = true;
        wheels[3].wheel->pressureAlertCallback(3);
    }else if(wheels[3].wheel->pressurePressed && getHeight({static_cast<float>(botBody.position.x - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::cos(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y - (WHEEL_OFFSET + WHEEL_DIAMETER/2)*std::sin(botBody.rotation*M_PI/180))}) < PUSH_SENSOR_HEIGHT){
        wheels[3].wheel->pressurePressed = false;
    }

    if(cameraLocked) cameraPosition = botBody.position;

    beacon.rotation += BEACON_ROTATION_RATE*360/1000*deltaTime;
    while(beacon.rotation > 360){
        beacon.rotation -= 360;
    }
}

void updateWindow(){
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "SpaceGrant Tester");
    drawing = true;
    while(drawing){
        update();

        sf::Event event{};
        while(window.pollEvent(event)){
            switch(event.type){
                case sf::Event::Closed:
                    drawing = false;
                    break;

                case sf::Event::KeyPressed:
                    if(event.key.code == sf::Keyboard::Space){
                        cameraLocked = !cameraLocked;
                    }
                    break;

                case sf::Event::MouseWheelScrolled:
                    cameraWidth *= (1 - ZOOM_AMOUNT*event.mouseWheelScroll.delta);
                    break;
            }
        }

        for(auto & obstacle : circleObstacles){
            obstacle.shape->setPosition((-cameraPosition.x + obstacle.position.x)*WINDOW_WIDTH/cameraWidth + WINDOW_WIDTH/2, (-cameraPosition.y - obstacle.position.y)*WINDOW_WIDTH/cameraWidth + WINDOW_HEIGHT/2);
            obstacle.shape->setRadius(obstacle.radius*WINDOW_WIDTH/cameraWidth);
            obstacle.shape->setOrigin(obstacle.shape->getRadius(), obstacle.shape->getRadius());

            window.draw(*obstacle.shape);
        }

        for(auto & obstacle : rectObstacles){
            obstacle.shape->setPosition((-cameraPosition.x + obstacle.position.x)*WINDOW_WIDTH/cameraWidth + WINDOW_WIDTH/2, (-cameraPosition.y - obstacle.position.y)*WINDOW_WIDTH/cameraWidth + WINDOW_HEIGHT/2);
            obstacle.shape->setSize({obstacle.size.x*WINDOW_WIDTH/cameraWidth, obstacle.size.y*WINDOW_WIDTH/cameraWidth});
            obstacle.shape->setOrigin(obstacle.size.x*WINDOW_WIDTH/(2*cameraWidth), obstacle.size.y*WINDOW_WIDTH/(2*cameraWidth));
            obstacle.shape->setRotation(-obstacle.rotation);

            window.draw(*obstacle.shape);
        }

        beacon.shape->setPosition((-cameraPosition.x + beacon.position.x)*WINDOW_WIDTH/cameraWidth + WINDOW_WIDTH/2, (-cameraPosition.y - beacon.position.y)*WINDOW_WIDTH/cameraWidth + WINDOW_HEIGHT/2);
        beacon.shape->setRadius(BEACON_DIAMETER/(2*cameraWidth)*WINDOW_WIDTH);
        beacon.shape->setOrigin(beacon.shape->getRadius(), beacon.shape->getRadius());
        beacon.directionLine[0].position = beacon.shape->getPosition();
        beacon.directionLine[1].position = {static_cast<float>(beacon.shape->getPosition().x + (BEACON_LENGTH*WINDOW_WIDTH/cameraWidth)*cos(beacon.rotation*M_PI/180)), static_cast<float>(beacon.shape->getPosition().y - (BEACON_LENGTH*WINDOW_WIDTH/cameraWidth)*sin(beacon.rotation*M_PI/180))};

        window.draw(*beacon.shape);
        window.draw(beacon.directionLine);

        for(auto & wheel : wheels){
            wheel.shape->setPosition((-cameraPosition.x + wheel.position.x)*WINDOW_WIDTH/cameraWidth + WINDOW_WIDTH/2, (-cameraPosition.y - wheel.position.y)*WINDOW_WIDTH/cameraWidth + WINDOW_HEIGHT/2);
            wheel.shape->setRadius(WHEEL_DIAMETER/(2*cameraWidth)*WINDOW_WIDTH);
            wheel.shape->setOrigin(wheel.shape->getRadius(), wheel.shape->getRadius());
            window.draw(*wheel.shape);

            wheel.directionLine[0].position = wheel.shape->getPosition();
            wheel.directionLine[1].position = {static_cast<float>(wheel.shape->getPosition().x + wheel.shape->getRadius()*cos((wheel.wheel->rotation + botBody.rotation)*M_PI/180)), static_cast<float>(wheel.shape->getPosition().y - wheel.shape->getRadius()*sin((wheel.wheel->rotation + botBody.rotation)*M_PI/180))};

            window.draw(wheel.directionLine);
        }

        botBody.shape->setPosition((-cameraPosition.x + botBody.position.x)*WINDOW_WIDTH/cameraWidth + WINDOW_WIDTH/2, (-cameraPosition.y - botBody.position.y)*WINDOW_WIDTH/cameraWidth + WINDOW_HEIGHT/2);
        botBody.shape->setSize({static_cast<float>(BODY_WIDTH*WINDOW_WIDTH/cameraWidth), static_cast<float>(BODY_WIDTH*WINDOW_WIDTH/cameraWidth)});
        botBody.shape->setOrigin(BODY_WIDTH*WINDOW_WIDTH/(2*cameraWidth), BODY_WIDTH*WINDOW_WIDTH/(2*cameraWidth));
        botBody.shape->setRotation(-botBody.rotation);

        if(sensor->scannerAngle >= -180){
            scannerLine[0].position = botBody.shape->getPosition();
            scannerLine[1].position = {static_cast<float>((sensor->scannerDistance*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + botBody.position.x - cameraPosition.x)*WINDOW_WIDTH/cameraWidth + WINDOW_WIDTH/2),
                                       static_cast<float>(-(sensor->scannerDistance*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180) + botBody.position.y - cameraPosition.y)*WINDOW_WIDTH/cameraWidth + WINDOW_HEIGHT/2)};

            window.draw(scannerLine);
        }

        window.draw(*botBody.shape);

        window.display();
        window.clear(sf::Color::Black);
    }
}

void setBotPosition(float x, float y){
    botBody.position = {x, y};
    wheels[0].position = {static_cast<float>(botBody.position.x - WHEEL_OFFSET*sin(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y + WHEEL_OFFSET*cos(botBody.rotation*M_PI/180))};
    wheels[1].position = {static_cast<float>(botBody.position.x + WHEEL_OFFSET*cos(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y + WHEEL_OFFSET*sin(botBody.rotation*M_PI/180))};
    wheels[2].position = {static_cast<float>(botBody.position.x + WHEEL_OFFSET*sin(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y - WHEEL_OFFSET*cos(botBody.rotation*M_PI/180))};
    wheels[3].position = {static_cast<float>(botBody.position.x - WHEEL_OFFSET*cos(botBody.rotation*M_PI/180)), static_cast<float>(botBody.position.y - WHEEL_OFFSET*sin(botBody.rotation*M_PI/180))};
}

Sensor::Sensor() {
    constructorUni();
}

Sensor::Sensor(uint8_t address, uint8_t bus) {
    constructorUni();
}

void Sensor::constructorUni() {
    scannerLine[0].color = sf::Color::Red;
    scannerLine[1].color = sf::Color::Red;

    botBody.shape = new sf::RectangleShape({BODY_WIDTH/INITIAL_WIDTH_DISTANCE*WINDOW_WIDTH, BODY_WIDTH/INITIAL_WIDTH_DISTANCE*WINDOW_WIDTH});
    botBody.shape->setFillColor(sf::Color::Cyan);
    botBody.shape->setOrigin(botBody.shape->getSize().x/2, botBody.shape->getSize().y/2);
    for(auto & wheel : wheels){
        wheel.shape = new sf::CircleShape(WHEEL_DIAMETER/(2*INITIAL_WIDTH_DISTANCE)*WINDOW_WIDTH);
        wheel.shape->setFillColor(sf::Color::Blue);
        wheel.shape->setOrigin(wheel.shape->getRadius(), wheel.shape->getRadius());

        wheel.directionLine = sf::VertexArray(sf::LinesStrip, 2);
    }

    Wheel1 = new Wheel(0, 0, 0);
    Wheel2 = new Wheel(1, 0, 0);
    Wheel3 = new Wheel(2, 0, 0);
    Wheel4 = new Wheel(3, 0, 0);

    wheels[0].wheel = Wheel1;
    wheels[1].wheel = Wheel2;
    wheels[2].wheel = Wheel3;
    wheels[3].wheel = Wheel4;

    beacon.shape = new sf::CircleShape(BEACON_DIAMETER/(2*INITIAL_WIDTH_DISTANCE)*WINDOW_WIDTH);
    beacon.shape->setFillColor(sf::Color::Green);
    beacon.shape->setOrigin(beacon.shape->getRadius(), beacon.shape->getRadius());
    beacon.rotation = 0;
    beacon.directionLine = sf::VertexArray(sf::LinesStrip, 2);
    beacon.directionLine[0].color = sf::Color::Green;
    beacon.directionLine[1].color = sf::Color::Green;

    std::ifstream ifs(CONFIG_NAME);
    json jf = json::parse(ifs);

    beacon.position = sf::Vector2<float>(jf["beacon"]["position"]["x"], jf["beacon"]["position"]["y"]);

    setBotPosition(jf["bot"]["position"]["x"], jf["bot"]["position"]["y"]);

    for(int i = 0; i < jf["obstacles"].size(); i++){
        if(jf["obstacles"][i]["type"] == "circle"){
            circleObstacles.emplace_back();

            circleObstacles.back().position = {jf["obstacles"][i]["x"], jf["obstacles"][i]["y"]};
            circleObstacles.back().radius = jf["obstacles"][i]["radius"];
            circleObstacles.back().height = jf["obstacles"][i]["height"];
            circleObstacles.back().shape = new sf::CircleShape(circleObstacles.back().radius*WINDOW_WIDTH/cameraWidth);
            circleObstacles.back().shape->setOrigin(circleObstacles.back().radius*WINDOW_WIDTH/cameraWidth, circleObstacles.back().radius*WINDOW_WIDTH/cameraWidth);

            if(jf["obstacles"][i]["height"] >= 0){
                circleObstacles.back().shape->setFillColor(sf::Color::Blue);

            }else{
                circleObstacles.back().shape->setFillColor(sf::Color::Red);
            }

        }else if(jf["obstacles"][i]["type"] == "rectangle"){
            rectObstacles.emplace_back();

            rectObstacles.back().position = {jf["obstacles"][i]["x"], jf["obstacles"][i]["y"]};
            rectObstacles.back().size = {jf["obstacles"][i]["width"], jf["obstacles"][i]["length"]};
            rectObstacles.back().rotation = jf["obstacles"][i]["rotation"];
            rectObstacles.back().height = jf["obstacles"][i]["height"];
            rectObstacles.back().shape = new sf::RectangleShape({rectObstacles.back().size.x*WINDOW_WIDTH/cameraWidth, rectObstacles.back().size.y*WINDOW_WIDTH/cameraWidth});
            rectObstacles.back().shape->setOrigin(rectObstacles.back().size.x*WINDOW_WIDTH/(2*cameraWidth), rectObstacles.back().size.y*WINDOW_WIDTH/(2*cameraWidth));

            if(jf["obstacles"][i]["height"] >= 0){
                rectObstacles.back().shape->setFillColor(sf::Color::Blue);

            }else{
                rectObstacles.back().shape->setFillColor(sf::Color::Red);
            }
        }
    }

    ifs.close();

    lastTime = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch()).count();

    std::thread drawThread(updateWindow);

    drawThread.detach();
}

Sensor::~Sensor() = default;

void asyncScanning(std::function<void(std::vector<RangeFinderPacket> &)> callbackFcn){
    std::vector<RangeFinderPacket> output;
    sensor->scannerAngle = -SCAN_ANGLE;
    for(uint8_t i = 0; i < SCAN_POINTS; i++){
        output.emplace_back();
        sensor->scannerDistance = SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180);

        double aScan = -std::tan((sensor->scannerAngle + botBody.rotation)*M_PI/180);
        double cScan = -(aScan*botBody.position.x + botBody.position.y);
        for(auto & obstacle : rectObstacles){
            // Bottom
            sf::Vector2<double> lineCenter = {obstacle.position.x + obstacle.size.y/2*std::sin(obstacle.rotation*M_PI/180), obstacle.position.y - obstacle.size.y/2*std::cos(obstacle.rotation*M_PI/180)};
            double a1 = -std::tan(obstacle.rotation*M_PI/180);
            double c1 = -(a1*lineCenter.x + lineCenter.y);
            sf::Vector2<double> intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
            double distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));

            if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.x/2 &&
                    distance < sensor->scannerDistance &&
                    std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
                    std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
                sensor->scannerDistance = distance;
            }


            // Right
            lineCenter = {obstacle.position.x + obstacle.size.x/2*std::cos(obstacle.rotation*M_PI/180), obstacle.position.y + obstacle.size.x/2*std::sin(obstacle.rotation*M_PI/180)};
            c1 = -(a1*lineCenter.x + lineCenter.y);
            intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
            distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));

            if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.y/2 &&
               distance < sensor->scannerDistance &&
               std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
               std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
                sensor->scannerDistance = distance;
            }


            // Top
            lineCenter = {obstacle.position.x - obstacle.size.y/2*std::sin(obstacle.rotation*M_PI/180), obstacle.position.y + obstacle.size.y/2*std::cos(obstacle.rotation*M_PI/180)};
            c1 = -(a1*lineCenter.x + lineCenter.y);
            intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
            distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));
            std::cout << "X: " << intersection.x << ", Y: " << intersection.y << std::endl;

            if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.x/2 &&
               distance < sensor->scannerDistance &&
               std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
               std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
                sensor->scannerDistance = distance;
            }


            // Left
            lineCenter = {obstacle.position.x - obstacle.size.x/2*std::cos(obstacle.rotation*M_PI/180), obstacle.position.y - obstacle.size.x/2*std::sin(obstacle.rotation*M_PI/180)};
            c1 = -(a1*lineCenter.x + lineCenter.y);
            intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
            distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));

            if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.y/2 &&
               distance < sensor->scannerDistance &&
               std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
               std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
                sensor->scannerDistance = distance;
            }
        }

        output.back().angle = sensor->scannerAngle;
        output.back().distance = std::sqrt(sensor->scannerDistance*sensor->scannerDistance + SCANNER_HEIGHT*SCANNER_HEIGHT)*1000;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        sensor->scannerAngle += 2.0*SCAN_ANGLE/SCAN_POINTS;
    }

    sensor->scannerAngle = -999;

    callbackFcn(output);
}

void getAngleAsync(float angle, std::function<void(RangeFinderPacket &)> callbackFcn){
    RangeFinderPacket output{};
    sensor->scannerAngle = angle;
    sensor->scannerDistance = SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180);

    double aScan = -std::tan((sensor->scannerAngle + botBody.rotation)*M_PI/180);
    double cScan = -(aScan*botBody.position.x + botBody.position.y);
    for(auto & obstacle : rectObstacles){
        // Bottom
        sf::Vector2<double> lineCenter = {obstacle.position.x + obstacle.size.y/2*std::sin(obstacle.rotation*M_PI/180), obstacle.position.y - obstacle.size.y/2*std::cos(obstacle.rotation*M_PI/180)};
        double a1 = -std::tan(obstacle.rotation*M_PI/180);
        double c1 = -(a1*lineCenter.x + lineCenter.y);
        sf::Vector2<double> intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
        double distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));

        if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.x/2 &&
           distance < sensor->scannerDistance &&
           std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
           std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
            sensor->scannerDistance = distance;
        }


        // Right
        lineCenter = {obstacle.position.x + obstacle.size.x/2*std::cos(obstacle.rotation*M_PI/180), obstacle.position.y + obstacle.size.x/2*std::sin(obstacle.rotation*M_PI/180)};
        c1 = -(a1*lineCenter.x + lineCenter.y);
        intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
        distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));

        if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.y/2 &&
           distance < sensor->scannerDistance &&
           std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
           std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
            sensor->scannerDistance = distance;
        }


        // Top
        lineCenter = {obstacle.position.x - obstacle.size.y/2*std::sin(obstacle.rotation*M_PI/180), obstacle.position.y + obstacle.size.y/2*std::cos(obstacle.rotation*M_PI/180)};
        c1 = -(a1*lineCenter.x + lineCenter.y);
        intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
        distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));
        std::cout << "X: " << intersection.x << ", Y: " << intersection.y << std::endl;

        if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.x/2 &&
           distance < sensor->scannerDistance &&
           std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
           std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
            sensor->scannerDistance = distance;
        }


        // Left
        lineCenter = {obstacle.position.x - obstacle.size.x/2*std::cos(obstacle.rotation*M_PI/180), obstacle.position.y - obstacle.size.x/2*std::sin(obstacle.rotation*M_PI/180)};
        c1 = -(a1*lineCenter.x + lineCenter.y);
        intersection = {(cScan-c1)/(a1-aScan), (c1*aScan-cScan*a1)/(a1-aScan)};
        distance = std::sqrt((intersection.x - botBody.position.x)*(intersection.x - botBody.position.x) + (intersection.y - botBody.position.y)*(intersection.y - botBody.position.y));

        if(std::sqrt(std::pow(intersection.x - lineCenter.x, 2) + std::pow(intersection.y - lineCenter.y, 2)) <= obstacle.size.y/2 &&
           distance < sensor->scannerDistance &&
           std::acos(((lineCenter.x - botBody.position.x)*std::cos((sensor->scannerAngle + botBody.rotation)*M_PI/180) + (lineCenter.y - botBody.position.y)*std::sin((sensor->scannerAngle + botBody.rotation)*M_PI/180))/distance) <= M_PI/2 &&
           std::tan(SCANNER_ANGLE*M_PI/180)*(SCANNER_HEIGHT/std::tan(SCANNER_ANGLE*M_PI/180) - distance) <= obstacle.height){
            sensor->scannerDistance = distance;
        }
    }

    output.angle = angle;
    output.distance = std::sqrt(sensor->scannerDistance*sensor->scannerDistance + SCANNER_HEIGHT*SCANNER_HEIGHT)*1000;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    sensor->scannerAngle = -999;

    callbackFcn(output);
}

int8_t Sensor::scan(std::function<void(std::vector<RangeFinderPacket> &)> callbackFcn) {
    std::thread scanThread(asyncScanning, callbackFcn);
    scanThread.detach();
    return 0;
}

int8_t Sensor::getAngle(float angle, std::function<void(RangeFinderPacket &)> callbackFcn) {
    std::thread angleThread(getAngleAsync, angle, callbackFcn);
    angleThread.detach();
    return 0;
}

float Sensor::getHeading() {
    float angle = 180*std::atan2(beacon.position.y - botBody.position.y, botBody.position.x - beacon.position.x)/M_PI + 180 + botBody.rotation;
    while(angle > 360){
        angle -= 360;
    }

    while(angle < 0){
        angle += 360;
    }
    return angle;
}

uint8_t Sensor::getRSSI() {
    return 0;
}

void Sensor::getHeadingRSSI(float &heading, uint8_t &rssi) {

}

void Sensor::intHandler(int gpio, int level, uint32_t tick) {

}

void wheelCallback(Wheel* caller, unsigned int time, const std::function<void(int8_t)>& callback){
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    caller->_state = INT_STATE_NONE;
    caller->running = STOPPED;
    caller->turning = STOPPED;
    callback(1);
}


Wheel::Wheel(uint8_t address, uint8_t bus, uint8_t interruptPin) {

}

Wheel::~Wheel() = default;

void Wheel::drive() {
    running = FORWARD;
}

void Wheel::stop(){
    running = STOPPED;
}

int8_t Wheel::move(float revolutions, const std::function<void(int8_t)>& callback) {
    if(_state != INT_STATE_NONE) return ERROR_BUSY;
    _state = INT_STATE;

    std::thread timerThread;

    if(revolutions >= 0){
        running = FORWARD;
        timerThread = std::thread(wheelCallback, this, revolutions*1000.0/SPEED, callback);
    }else{
        running = BACKWARD;
        timerThread = std::thread(wheelCallback, this, -revolutions*1000.0/SPEED, callback);
    }
    timerThread.detach();
    return 0;
}

float Wheel::getPosition() {
    return position;
}

int8_t Wheel::turnWheel(float degrees, const std::function<void(int8_t)>& callback) {
    if(_state != INT_STATE_NONE) return ERROR_BUSY;
    _state = INT_STATE;

    std::thread timerThread;

    if(degrees >= 0){
        turning = FORWARD;
        timerThread = std::thread(wheelCallback, this, degrees*1000.0/(360*TURN_SPEED), callback);
    }else{
        turning = BACKWARD;
        timerThread = std::thread(wheelCallback, this, -degrees*1000.0/(360*TURN_SPEED), callback);
    }

    timerThread.detach();
    return 0;
}

int8_t Wheel::resetRotation(std::function<void(int8_t)> callback) {
    if(_state != INT_STATE_NONE) return ERROR_BUSY;
    _state = INT_STATE;

    float degrees;
    if(rotation > 0){
        degrees = 360 - rotation;
    }else{
        degrees = -rotation;
    }

    turning = FORWARD;
    std::thread timerThread = std::thread(wheelCallback, this, degrees*1000.0/(360*TURN_SPEED), callback);
    timerThread.detach();
    return 0;
}

int8_t Wheel::setRotation(float degrees, std::function<void(int8_t)> callback) {
    if(_state != INT_STATE_NONE) return ERROR_BUSY;
    _state = INT_STATE;


    degrees = dist(rotation, degrees, 360);
    if(degrees >= 0){
        turning = FORWARD;
    }else{
        degrees *= -1;
        turning = BACKWARD;
    }

    std::thread timerThread = std::thread(wheelCallback, this, degrees*1000.0/(360*TURN_SPEED), callback);
    timerThread.detach();
    return 0;
}

float Wheel::getRotation() const{
    return rotation;
}

void Wheel::setPressureAlertFunction(std::function<void(uint8_t)> callback) {
    pressureAlertCallback = std::move(callback);
}

bool Wheel::getPressureSensor() const {
    return pressurePressed;
}

void Sensor::setGyroAlertFunction(std::function<void(Vector3)> callback, float angle) {
    gyroAlertCallback = std::move(callback);
    gyroAlertAngle = angle;
}

Vector3 Sensor::getGyro() {
    Vector3 output;
    output.z = botBody.rotation;
    output.y = 180/M_PI*(-std::atan2(getHeight(wheels[1].position), WHEEL_OFFSET) + std::atan2(getHeight(wheels[3].position), WHEEL_OFFSET))/2;
    output.x = 180/M_PI*(std::atan2(getHeight(wheels[0].position), WHEEL_OFFSET) - std::atan2(getHeight(wheels[2].position), WHEEL_OFFSET))/2;
    return output;
}


