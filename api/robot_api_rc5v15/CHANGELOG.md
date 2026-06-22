# Changelog

> Все значимые изменения в проекте документируются в этом файле.
> Формат основан на [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
> и этот проект придерживается [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

- [Changelog](#changelog)
  - [\[1.5.0\]](#150)
  - [\[1.4.2\] - 2026-03](#142---2026-03)
  - [\[1.4.1\] - 2025-12](#141---2025-12)
  - [\[1.4.0\] - 2025-11](#140---2025-11)
  - [\[1.3.8\] - 2025-06](#138---2025-06)
  - [\[1.3.7\] - 2024-12](#137---2024-12)
  - [\[1.3.6\] - 2024-08](#136---2024-08)
  - [\[1.3.5\]](#135)
  - [\[1.3.4\]](#134)
  - [\[1.3.3\]](#133)


## [1.5.0]

### Added
- Обновлен протокол обмена до 600 (поддерживаемая версия ядра 1.5.0 и выше)
- Совместимость API с версиями интерпретатора python 3.8.x и 3.9.х (для RoboDK)
- Поддержка управления роботом в реальном времени с помощью подмодуля
`RobotApi.motion.realtime`.
- Поддержка моделей роботов RC20 и RC30.
- Метод получения максимально допустимой полезной нагрузки робота с помощью
метода `RobotApi.payload.get_max`.
- Получение конфигурационных параметров робота с помощью подкласса
`RobotApi.configuration`.
- Работа с параметрами пределов безопасности робота с помощью подкласса
`RobotApi.safety.limits`.
- Метод генерации одиночного импульса на цифровом выходе
`RobotApi.io.digital.pulse_output`.
- Метод генерации одиночного импульса на цифровом выходе запястья
`RobotApi.wrist.digital.pulse_output`.
- Метод получения мгновенных скоростей робота
`RobotApi.motion.get_actual_velocity`.
- Метод получения количества точек в буфере
`RobotApi.motion.get_waypoint_buffer_size`.
- Класс `API.tools.PeriodicPublisher` для оформления "подписки" на результат
выполнения любой функции.
- В конструктор класса `RobotApi` добавлен параметр `allow_rtd_reconnection`,
разрешающий автоматическое переподключение RTD. Позволяет повысить
стабильность работы АПИ в ситуациях, когда переподключение RTD допустимо.
- Поддержка выполнения команд G кода, включает в себя:
  - интерпретатор G кода в команды управления роботом
(класс `API.tools.GCodeExecutor`);
  - метод `API.tools.gcode_val_to_meters` для перевода величин команд G кода в
метры, метры в секунду для написания пользовательских обработчиков команд;
  - сущности, необходимые для написания пользовательских обработчиков команд:
`API.types.GCodeCommand`, `API.types.GCodeExecutionContext`,
`API.types.GCodeState` и `API.types.GCodeHandlerType`.
- Инструмент `API.tools.calibrate_tcp` для калибровки параметров ЦТИ по 4 и
более точек.

### Changed
- Поддерживаемая версия ядра 1.5.0 и выше
- Подклассы `RobotApi.controller_state` и `RobotApi.controller_gravity`
перенесены в подклассы `RobotApi.controller.state` и
`RobotApi.controller.gravity` соответственно с поддержкой обратной совместимости
- Подкласс `RobotApi.safety_status` перенесен в подкласс
`RobotApi.safety.status` и с поддержкой обратной совместимости
- В методах `API.tools.load_impulse_vars` и `API.tools.save_impulse_vars`
добавлен необязательный параметр `target`, позволяющий указать используемый
прокси-объект для хранения загружаемых переменных
- Доработан механизм установки стандартных параметров движения (методом
`set_motion_config`)
- Переработана структура организации команд контроллера
- Уменьшено время, необходимое для подключения к роботу в режиме "full"
- Тип параметра `await_sec`  всех функций АПИ изменен с `int` на `int | float`.

### Fixed
- Исправлено сохранение переменных ПО Пульс в режиме совместимости

### Deprecated
- Подклассы `RobotApi.controller_state` и `RobotApi.controller_gravity`
перенесены в подклассы `RobotApi.controller.state` и
`RobotApi.controller.gravity` соответственно с поддержкой обратной совместимости
- Подкласс `RobotApi.safety_status` перенесен в подкласс
`RobotApi.safety.status` и с поддержкой обратной совместимости


## [1.4.2] - 2026-03

### Added
- Совместимость API с версиями интерпретатора python 3.14.x
- Удобная система импортов из модуля API, добавлено 4 подмодуля:
  - `coords`
  - `types`
  - `tools`
  - `io`
- Реализованы импорты:
  - `API.RobotAPI` вместо `API.rc_api.RobotAPI`
  - `API.coords.CoordinateSystem` вместо `API.source.features.mathematics.coordinate_system.CoordinateSystem`
  - `API.coords.calculate_plane_from_points` вместо `API.source.features.mathematics.coordinate_system.calculate_plane_from_points`
  - `API.coords.convert_position_orientation` вместо `API.source.features.mathematics.coordinate_system.convert_position_orientation`
  - `API.types.RobotInfo` вместо `API.source.models.classes.data_classes.api_version.RobotInfo`
  - `API.types.DhModelParams` вместо `API.source.models.classes.data_classes.service_types.DhModelParams`
  - `API.types.JointAngleDiscrepancy` вместо `API.source.models.classes.data_classes.service_types.JointAngleDiscrepancy`
  - `API.types.CoordinateSystemInfoType` вместо `API.source.models.classes.enum_classes.various_types.CoordinateSystemInfoType`
  - типы данных `AnalogIndex`, `AngleUnits`, `CompareSigns`, `ControllerStateName`,
`DigitalIndex`, `DigitalSafetyIndex`, `DigitalWristIndex`, `InputFunctionName`,
`JogAxis`, `JogDirection`, `JointIndex`, `MotionModeName`, `OutputFunctionName`,
`PositionFormat`, `PositionOrientation`, `PowerUnits`, `ReferenceFrame`,
`SafetyStatus`, `WristInputActivationType`, `WristModeName` в подмодуле
`API.types` вместо `API.source.models.type_aliases`
  - `API.tools.sleep` вместо `API.source.features.tools.sleep`
- Возможность использовать конструктор класса `RobotAPI` с контекстным
менеджером `with`
- Возможность изменять статус подключения к роботу с помощью контекстного
менеджера `with` благодаря методу `RobotAPI.connected`
- Возможность временно изменять параметры нагрузки с помощью контекстного
менеджера `with` с помощью метода `RobotAPI.payload.using`
- Возможность проверить достижимость точки с помощью метода `RobotApi.motion.is_point_reachable`
- Параметр `coordinate_system` для явного указания системы координат (вместо
передачи положения через метод `convert_position_orientation`) добавлен в методы:
  - `RobotApi.motion.linear.add_new_waypoint`
  - `RobotApi.motion.joint.add_new_waypoint`
  - `RobotApi.motion.advanced.add_movel_waypoint`
  - `RobotApi.motion.advanced.add_movep_waypoint`
  - `RobotApi.motion.advanced.add_movec_waypoint`
  - `RobotApi.motion.kinematics.get_inverse`
- Возможность автоматической подстановки системы координат без явного указания
в методы, где она используется, с помощью контекстного менеджера `with` и метода
`CoordinateSystem.in_frame`
- Инструменты для более удобной работы с системами координат:
  - копирование системы координат, метод `CoordinateSystem.copy`
  - изменение единиц измерения углов, метод `CoordinateSystem.with_units`
  - смещение цента системы координат, метод `CoordinateSystem.offset`
  - поворот системы координат, метод `CoordinateSystem.rotate`
  - сравнение систем координат, метод `CoordinateSystem.is_close`
  - вычисление расстояния между центрами систем координат, метод `CoordinateSystem.distance_to`
  - определение ориентации системы координат по вектору, метод `CoordinateSystem.align_with_vector`
- Подмодуль `RobotApi.diagnostics` для получения диагностической информации
о роботе. Включает методы:
  - `get_controller_temperature`
  - `get_robot_voltage`
  - `get_robot_current`
  - `get_io_current`
  - `get_tool_current`
  - `get_joints_motor_temperatures`
  - `get_joints_controller_temperatures`
  - `get_joints_currents`
  - `get_joints_voltages`
  - `get_joints_torques`
- Вывод полной структуры RTD при получении некорректных состояний в StateHandler
- В подкласс аналоговых входов запястья добавлены методы:
  - `RobotApi.wrist.analog.configure_input`
  - `RobotApi.wrist.analog.get_input_in_units`
  - `RobotApi.wrist.analog.wait_input_in_units`
- Подкласс запястья `RobotApi.wrist.rs_485`. Включает методы:
  - `connect` - установить соединение
  - `connected` - установить соединение с помощью контекстного менеджера `with`
  - `is_connected` - получить статус соединения
  - `reset` - отправить команду сброса
  - `read` - прочитать данные из канала связи
  - `write` - отправить данные в канал связи
  - `query` - отправить данные в канал связи и прочитать ответ (`write` + `read`)
  - `get_status` - получить статус соединения
  - `clear_buffer` - отчистить внутренний буфер
  - `disconnect` - разорвать соединение
- Клиентами для промышленных интерфейсов в подмодуль `API.io`:
  - `WristRs485` - "сырой" RS-485 интерфейс платы запястья
  - `WristModbusRS485Client` - надстройка Modbus над `WristRs485`
- Структуры `Response` и `Rs485ReturnCodes` в подмодуль `API.types`
- Методы `add_movej_waypoint` и `add_movej_tcp_waypoint` в подкласс
`RobotApi.motion.advanced`
- Инструменты для работы с переменными "Импульса" при запуске скрипта в режиме
совместимости ПО "Импульс" (запуске скрипта через команду ПО "Импульс"):
  - `load_impulse_vars`
  - `save_impulse_vars`
  - `send_error_to_impulse`
  - `impulse_vars`
  - `ImpulseVarsProxy`

### Changed
- Поддерживаемая версия ядра 1.4.3 и выше
- Переработан интерфейс `Simple joystick`
- Метод `RobotApi.wrist.get` переименован в `RobotApi.wrist.get_mode`
- Метод `RobotApi.wrist.set` переименован в `RobotApi.wrist.set_mode`
- У метода `RobotApi.wrist.analog.get_input` убран параметр `units` и изменен
формат возвращаемых данных
- У метода `RobotApi.wrist.analog.wait_input` убран параметр `units`

### Fixed
- Увеличена стабильность приема RTD данных
- Некорректное интерпретирование state из RTD данных в ряде случаев

### Removed
- Метод `RobotApi.wrist.analog.check_wrist_enable`


## [1.4.1] - 2025-12

### Added
- Совместимость API с версиями интерпретатора python 3.13.x
- Метод получения текущей позиции робота (get_actual_position) в подклассе
RobotAPI.motion
- Метод получения последней сохраненной в контроллере позиции робота
(get_last_saved_position) в подклассе RobotAPI.motion
- Сервисный класс, хранящий состояние подключения к роботу, и обработчик
функций на его основе
- Плавное переключение между режимами подключения `read only` и `full`
- DH параметры робота в рамках возвращаемой информации в методе
`RobotAPI.get_robot_info`

### Changed
- Вызов методов API сопровождается проверкой возможности их вызова на основе
состояния подключения к роботу
- Все `read` методы теперь доступны при подключении в режиме `read only`
- Прием RTD и проверка статуса контроллера вынесены в отдельные потоки (было два
потока в составе одного ThreadPool)
- Блокирующие методы с ожиданием завершают работу при потере соединения с роботом
- Импорт фреймворка `tkinter` для интерфейса `Simple joystick` сделан "ленивым"
для совместимости с системами, на которых не установлен `tkinter`
- Сборка пакета полностью переведена с `setuptools` на `hatchling`
- Метод `set_jog_param_in_tcp` перенесен из `motion.joint` в `motion.linear` с
изменением интерфейса (без обратной совместимости)
- Расширена документация всех методов API

### Removed
- Библиотека работы с глобальными переменными (директория `./lib`)

## [1.4.0] - 2025-11

### Added
- Поддержка Advanced motion (RPMP) без точек типа joint
- Возможность одновременного указания координат ЦТИ и углов поворотов звеньев в
методе добавления точки joint motion
- Возможность установки обработчика для подтверждения положения робота при
рассогласовании считываемых и последних сохраненных углов
- Возможность контролировать подключение и отключение от контроллера, выполнять
повторное подключение для одного экземпляра класса
- Возможность проверять статус подключения к контроллеру (активно или нет)
- Возможность установки функции-обработчика, которая будет вызвана при
незапланированном обрыве соединения с контроллером робота

### Changed
- Все ошибки (exceptions) API наследуются от базовой ошибки ApiError


## [1.3.8] - 2025-06

### Added
- Поддержка 500-го протокола
- Поддержка платы инструмента
- Функции входов для запястья
- Метод установки и получения вектора гравитации для контроллера

### Fixed
- Мелкие правки и улучшения


## [1.3.7] - 2024-12

### Added
- Метод ожидания любого входного цифрового сигнала wait_any_input()
- Режим read_only для подключения в режиме read only к порту RTD
- Методы работы с домашней позицией: get_home_pose(), set_home_pose(),
move_to_home_pose()


## [1.3.6] - 2024-08

### Added
- Метод проверки состояния исполнения текущих заданных точек
check_waypoint_completion()
- Метод получения текущего двоичного значения на цифровом входе безопасности
get_safety_input()
- Метод получения всех функций входов безопасности get_safety_input_functions()
- В Класс RobotAPI добавлен аргумент ignore_controller_exceptions для
игнорирования ошибок состояний контроллера робота


## [1.3.5]

### Added
- Убрана задержка при движении в simple_joystick
- В simple_joystick добавлена кнопка "Вставить" для вставки скопированных
координат в поле перемещения
- Убрана привязка клавиш simple_joystick

### Changed
- Метод получения решения обратной задачи кинематики get_inverse(), теперь
оптимальное решение рассчитывается на стороне ядра

### Fixed
- Метод для получения сигналов с цифровых входов get_input()


## [1.3.4]

### Changed
- Версия ядра обновлена до 1.2.603.

### Fixed
- Корректное отображение сообщений об ошибках
- Обработка вызванных исключений
- Метод глобальных настроек робота set_motion_config()


## [1.3.3]

### Added
- В simple_joustick добавлено движение типа смещения offset

### Changed
- Перед добавлением точки типа tcp_pose больше не требуется ждать завершения
движения робота,  данные точки теперь можно добавлять в очередь наравне с остальными.