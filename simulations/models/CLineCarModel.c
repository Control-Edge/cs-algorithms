#include "CLineCarModel.h"

#include <math.h>

static double clamp(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

static void update_sensors(LineCarModel *model, double curvature);
static double curvature_at(LineCarModel *model, double distance);

void line_car_model_reset(LineCarModel *model)
{
    model->battery_factor = 1.0;
    model->speed_left = 0.0;
    model->speed_right = 0.0;
    model->distance = 0.0;
    model->lateral_offset = 0.0;
    model->heading_error = 0.0;
    model->line_offset_at_sensors = 0.0;
    model->segment = 0;
    model->finished = 0;
    update_sensors(model, curvature_at(model, 0.0));
}

double line_car_model_track_length(const LineCarModel *model)
{
    double length = 0.0;
    for (int i = 0; i < model->config.segment_count; i++) {
        length += model->config.track[i].length;
    }
    return length;
}

static void update_sensors(LineCarModel *model, double curvature)
{
    const LineCarModelConfig *cfg = &model->config;

    /* Line position under the sensor array: the array sits ahead of the
       axle, and in a curve the line bends away by about k*L^2/2 there */
    model->line_offset_at_sensors = -(model->lateral_offset + cfg->sensor_ahead * sin(model->heading_error))
                                    + 0.5 * curvature * cfg->sensor_ahead * cfg->sensor_ahead;

    const double center = (LINE_CAR_SENSOR_COUNT - 1) / 2.0;
    for (int i = 0; i < LINE_CAR_SENSOR_COUNT; i++) {
        double sensor_position = (i - center) * cfg->sensor_pitch;
        model->sensors.ir[i] = fabs(sensor_position - model->line_offset_at_sensors) < 0.5 * cfg->line_width;
    }
}

static double curvature_at(LineCarModel *model, double distance)
{
    double start = 0.0;
    for (int i = 0; i < model->config.segment_count; i++) {
        if (distance < start + model->config.track[i].length) {
            model->segment = i;
            return model->config.track[i].curvature;
        }
        start += model->config.track[i].length;
    }
    model->segment = model->config.segment_count - 1;
    model->finished = 1;
    return 0.0;
}

void line_car_model_cyclic(LineCarModel *model, double dt)
{
    const LineCarModelConfig *cfg = &model->config;

    /* Motors: first-order lag towards the commanded speed */
    double target_left = clamp(model->actuators.pwm_left, -1.0, 1.0) * cfg->max_speed * model->battery_factor;
    double target_right = clamp(model->actuators.pwm_right, -1.0, 1.0) * cfg->max_speed * model->battery_factor;
    model->speed_left += (dt / cfg->motor_tau) * (target_left - model->speed_left);
    model->speed_right += (dt / cfg->motor_tau) * (target_right - model->speed_right);

    double speed = 0.5 * (model->speed_left + model->speed_right);
    double yaw_rate = (model->speed_right - model->speed_left) / cfg->wheelbase;

    /* Pose relative to the line */
    double curvature = curvature_at(model, model->distance);
    double along = speed * cos(model->heading_error) / (1.0 - curvature * model->lateral_offset);
    model->distance += along * dt;
    model->lateral_offset += speed * sin(model->heading_error) * dt;
    model->heading_error += (yaw_rate - curvature * along) * dt;
    update_sensors(model, curvature_at(model, model->distance));
}
