/*
 * 0) в папке LORR24_SAVE лежит исходная версия до чистки репозитория. Там лежит старая папка Data до этих +gg
 *    в папке LORR24_origin лежит исходная папка проекта, но уже после чистки, там что-то поломалось, поэтому мы тут
 *
 * 1) попробовать расширить рекурсию EPIBT, чтобы при выборе операции она могла коллизить с двумя и более агентами
 *    но тогда из-за увеличения дерева рекурсии нужно более грамотно это отсекать
 *    например можно разрешить коллизии только при текущей глубине рекурсии = 1
 *
 * 2) попробовать шафлить порядок агентов для PEPIBT+LNS в каждой инстанции EPIBT+LNS
 * 3) запустить какой-то один большой тест с разным depth для EPIBT и EPIBT+LNS
 */

КОГДА БУДУ ТЕСТИРОВАТЬ, ТО НУЖНО ПОСТАВИТЬ time_limit = 1000, потому что я системе даю 2000
