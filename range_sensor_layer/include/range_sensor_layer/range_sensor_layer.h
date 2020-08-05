// Copyright 2018 David V. Lu!!
#ifndef RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H_
#define RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <sensor_msgs/Range.h>
#include <range_sensor_layer/RangeSensorLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <list>
#include <string>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace range_sensor_layer
{

class RangeSensorLayer : public costmap_2d::CostmapLayer
{
public:
  enum InputSensorType
  {
    VARIABLE,
    FIXED,
    ALL
  };

  RangeSensorLayer();

  /** @brief This is called at the end of initialize().  Override to
   * implement subclass-specific initialization.
   *
   * tf_, name_, and layered_costmap_ will all be set already when this is called. */
  // Инициализация, подписка на топики, парсинг входных данных, подклюение Dynamic Reconfigure
  virtual void onInitialize();

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  // Обнновление положение датчика, костмапы, мин и макс расстояний приема
  // Варнинг если установлен таймаут на данные в параметре, вызов resetRange и updateCostmap
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  // Обновляет стоимости в поданном квадрате. Ниже clear_thre => чистит. Выше mark_thre => наносит
  // Обновляет буфер прошлого значения ячейки костмапа
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  // Перезагрузка слоя: вызов deactivate() и activate()
  virtual void reset();

  /** @brief Stop publishers. */
  // Чистит буфер сообщений
  virtual void deactivate();

  /** @brief Restart publishers if they've been stopped. */
  // Чистит буфер сообщений
  virtual void activate();

private:
  // Обновление основных переменных при диагностической реконфигурации
  void reconfigureCB(range_sensor_layer::RangeSensorLayerConfig &config, uint32_t level);

  // буферизация входного сообщения расстояния
  void bufferIncomingRangeMsg(const sensor_msgs::RangeConstPtr& range_message);

  // обработка входного сообщения типа Range
  void processRangeMsg(sensor_msgs::Range& range_message);

  // обработка входного сообщения типа Range при max_range == min_range,
  // вызывает updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone) в коцне работы
  void processFixedRangeMsg(sensor_msgs::Range& range_message);

  // обработка входного сообщения типа Range при max_range > min_range,
  // вызывает updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone) в коцне работы
  void processVariableRangeMsg(sensor_msgs::Range& range_message);

  // сброс показаний о минимальном и максимальном расстояниях
  void resetRange();

  // вызывается в updateBounds()
  // обновление костмапа показаниями из буфера сообщений
  // вызываетс processRangeMessageFunc_() для обработки сообщений в конце
  void updateCostmap();

  // обновляет костмап при полученном сообщении
  // поялвяется в конце обработчиков сообщений
  //
  // последовательность вызовов такова:
  // updateBounds -> updateCostmap() -> processRangeMessageFunc_() -> processRangeMsg / processFixedRangeMsg / processVariableRangeMsg
  //  -> updateCostmap(перегрузка с входными переменными, то есть версия ниже) -> updateCell()
  void updateCostmap(sensor_msgs::Range& range_message, bool clear_sensor_cone);

  // Используется для математики в sensor_model()
  double gamma(double theta);
  double delta(double phi);

  // Вычисление вероятности того, что датчик верно определил перед собой препятствие
  double sensor_model(double r, double phi, double theta);

  // Место использования не найдено
  void get_deltas(double angle, double *dx, double *dy);

  // Обновление ячейки костмапа
  void update_cell(double ox, double oy, double ot, double r, double nx, double ny, bool clear);

  // Перевод из цифровых значений костмапа в вероятность препятствия ( 128 -> 0.5 )
  double to_prob(unsigned char c)
  {
    return static_cast<double>(c) / costmap_2d::LETHAL_OBSTACLE;
  }

  // Перевод из вероятности препятствия в цифровые значения костмапа ( 0.5 -> 128 )
  unsigned char to_cost(double p)
  {
    return static_cast<unsigned char>(p * costmap_2d::LETHAL_OBSTACLE);
  }

  // Обработка сообщений
  boost::function<void(sensor_msgs::Range& range_message)> processRangeMessageFunc_;

  // Мьютекс (англ. mutex, от mutual exclusion — «взаимное исключение») — это базовый механизм синхронизации.
  // Он предназначен для организации взаимоисключающего доступа к общим данным для нескольких потоков с
  // использованием барьеров памяти (для простоты можно считать мьютекс дверью, ведущей к общим данным).
  boost::mutex range_message_mutex_;

  // Буфер сообщений с типа Range с показаниями сенсора
  std::list<sensor_msgs::Range> range_msgs_buffer_;

  // Угол обзора сенсора / 2
  double max_angle_;

  // Значение Phi
  // Во ВХОДЫХ ПАРАМЕТРАХ под названием phi
  // Значение по умолчанию: 1.2
  double phi_v_;

  // Inflate the triangular area covered by the sensor (percentage)
  // Во ВХОДЫХ ПАРАМЕТРАХ под названием inflate_cone
  // Значение по умолчанию: 1
  double inflate_cone_;

  // Переменная для хранения названия глобальной системы координат
  std::string global_frame_;

  // Probability below which cells are marked as free
  // Во ВХОДЫХ ПАРАМЕТРАХ под названием clear_threshold
  // Значение по умолчанию: 0.2
  double clear_threshold_;

  // Probability above which cells are marked as occupied
  // Во ВХОДЫХ ПАРАМЕТРАХ под названием mark_threshold
  // Значение по умолчанию: 0.8
  double mark_threshold_;

  // Clear on max reading
  // Во ВХОДЫХ ПАРАМЕТРАХ под названием clear_on_max_reading
  // Значение по умолчанию: False
  bool clear_on_max_reading_;

  // No Readings Timeout
  // Во ВХОДЫХ ПАРАМЕТРАХ под названием
  // Значение по умолчанию: 0.0
  double no_readings_timeout_;

  // Время с последнего вызова updateCostmap(range_message, clear_sensor_cone) и получения данных сенсора
  // Используется для детектирования отсуствия входных данных при включенном параметре no_readings_timeout
  ros::Time last_reading_time_;

  // Количество буферизованных сообщений
  unsigned int buffered_readings_;

  // Список топиков с показаниями сенсоров
  std::vector<ros::Subscriber> range_subs_;

  // Минимальные координаты квадрата костмапас
  double min_x_, min_y_;

  // Максимальные координаты квадрата костмапас
  double max_x_, max_y_;

  // Сервис для динамической реконфигурации переменных
  dynamic_reconfigure::Server<range_sensor_layer::RangeSensorLayerConfig> *dsrv_;

  // Determine barycentric coordinates
  // Используется updateCostmap(range_message, clear_sensor_cone)
  float area(int x1, int y1, int x2, int y2, int x3, int y3)
  {
    return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
  };

  // Barycentric coordinates inside area threshold; this is not mathematically sound at all, but it works!
  // Используется updateCostmap(range_message, clear_sensor_cone)
  int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy)
  {
    return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
  };
};
}  // namespace range_sensor_layer
#endif  // RANGE_SENSOR_LAYER_RANGE_SENSOR_LAYER_H
