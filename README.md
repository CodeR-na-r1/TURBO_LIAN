Алгоритм для построения пути с параметрами ограничения угла
Настраивается при помощи констант K_ANGLE и K_DELTA в файле Lian.hpp.
Также поведение алгоритма можно менять переключением строчек 84 и 85 в функции Expand (файл LianFunctions.hpp)
Для запуска необходимо установить библиотеку opencv в корневую паку проекта, а также положить в корневую папку dll файл opencv_world480.dll (или opencv_world480d.dll для запуска в режиме debug)