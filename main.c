#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>

#define UTIL_ALLOC(type, count) (type*) calloc(count, sizeof(type));

#define UTIL_REALLOC(pointer, type, count) ((type*) realloc(pointer, sizeof(type) * count))

#define UTIL_CHECK(pointer) if(pointer == NULL) printf("error: file %s: line %d: pointer was NULL", __FILE__, __LINE__), exit(1)

#define UTIL_LEN(array) ((int32_t) (sizeof(array) / sizeof(*array)))

typedef struct
{
    int32_t x;
    int32_t y;
}
Point;

typedef struct
{
    Point* point;
    int32_t start;
    int32_t end;
    int32_t max;
}
Queue;

typedef struct
{
    char* object;
    int32_t rows;
    int32_t cols;
}
Field;

static int32_t Util_Time(void)
{
    struct timeval stamp;
    gettimeofday(&stamp, NULL);
    return stamp.tv_sec * (int) 1e6 + stamp.tv_usec;
}

static char* Util_StringDup(const char* string)
{
    const int32_t len = strlen(string) + 1;
    char* const dup = UTIL_ALLOC(char, len);
    UTIL_CHECK(dup);
    strcpy(dup, string);
    return dup;
}

static Point Point_Add(const Point a, const Point b)
{
    const Point out = { a.x + b.x, a.y + b.y };
    return out;
}

static bool Point_Equal(const Point a, const Point b)
{
    return a.x == b.x && a.y == b.y;
}

static Queue Queue_Make(const int32_t max)
{
    Point* point = UTIL_ALLOC(Point, max);
    UTIL_CHECK(point);
    Queue queue = { point, 0, 0, max };
    return queue;
}

static Queue Queue_Enqueue(Queue queue, const Point point)
{
    if(queue.end == queue.max)
    {
        queue.max *= 2;
        Point* const temp = UTIL_REALLOC(queue.point, Point, queue.max);
        UTIL_CHECK(temp);
        queue.point = temp;
    }
    queue.point[queue.end++] = point;
    return queue;
}

static Queue Queue_Dequeue(Queue queue, Point* point)
{
    *point = queue.point[queue.start++];
    return queue;
}

static void Queue_Free(Queue queue)
{
    free(queue.point);
}

static bool Queue_IsEmpty(Queue queue)
{
    return queue.start == queue.end;
}

static bool Field_InBounds(const Field field, const Point point)
{
    return point.x < field.cols && point.x >= 0
        && point.y < field.rows && point.y >= 0;
}

static bool Field_IsWalkable(const Field field, const Point point)
{
    return field.object[point.x + point.y * field.cols] == ' ';
}

static Queue Field_SearchBreadthFirst(const Field field, const Point start, const Point goal)
{
    Queue frontier = Queue_Make(8);
    frontier = Queue_Enqueue(frontier, start);
    Queue came_from = Queue_Make(8);
    const Point none = { -1, -1 };
    for(int32_t i = 0; i < field.rows * field.cols; i++)
        came_from = Queue_Enqueue(came_from, none);
    while(!Queue_IsEmpty(frontier))
    {
        Point current;
        frontier = Queue_Dequeue(frontier, &current);
        if(Point_Equal(current, goal))
            break;
        const Point deltas[] = {
            { -1, +1 }, { 0, +1 }, { 1, +1 },
            { -1,  0 }, /* ---- */ { 1,  0 },
            { -1, -1 }, { 0, -1 }, { 1, -1 },
        };
        for(int32_t i = 0; i < UTIL_LEN(deltas); i++)
        {
            const Point next = Point_Add(current, deltas[i]);
            if(Field_InBounds(field, next)
            && Field_IsWalkable(field, next))
            {
                if(Point_Equal(came_from.point[next.x + next.y * field.cols], none))
                {
                    frontier = Queue_Enqueue(frontier, next);
                    came_from.point[next.x + next.y * field.cols] = current;
                }
            }
        }
    }
    Queue path = Queue_Make(8);
    Point current = goal;
    while(!Point_Equal(current, start))
    {
        path = Queue_Enqueue(path, current);
        current = came_from.point[current.x + current.y * field.cols];
    }
    path = Queue_Enqueue(path, start);
    Queue_Free(frontier);
    Queue_Free(came_from);
    return path;
}

static Field Field_Trace(Field field, const Queue path)
{
    for(int32_t i = path.start; i < path.end; i++)
    {
        const Point point = path.point[i];
        field.object[point.x + field.cols * point.y] = '*';
    }
    return field;
}

static void Field_Print(const Field field)
{
    for(int32_t y = 0; y < field.rows; y++)
    {
        for(int32_t x = 0; x < field.cols; x++)
            printf("%c", field.object[x + field.cols * y]);
        putchar('\n');
    }
}

static Field Field_Make(void)
{
    const char* const object =
        "###################################################"
        "#                                       #         #"
        "#    ###############################    #    #    #"
        "#    #                             #    #    #    #"
        "#    #    #####################    #    #    #    #"
        "#    #    #                   #    #    #    #    #"
        "#    #    #####     #####     #    #    #    #    #"
        "#    #              #         #    #    #    #    #"
        "#    #    #####################    #    ######    #"
        "#    #    #                   #    #              #"
        "#    ######    ###########    #    ###########    #"
        "#         #    #         #    #              #    #"
        "#    #    #    #    #    #    #     ##########    #"
        "#    #    #    #    #    #    #                   #"
        "######    #    #    #    #    #    ################"
        "#         #    #    #    #    #    #              #"
        "#    #    #    ######    #    #    #     #####    #"
        "#    #    #              #    #    #         #    #"
        "#    ###############     #    #    ###########    #"
        "#                        #    #                   #"
        "###################################################";

    const Field field = {
        Util_StringDup(object), 21, 51 // NOTE: Do not forget to set these dimensions when you change the map size!
    };
    return field;
}

static void Field_Free(Field field)
{
    free(field.object);
}

int main(void)
{
    Field field = Field_Make();
    const Point start = { 18, 14 };
    const Point goal = { 28, 7 };

    const int32_t t0 = Util_Time();
    const Queue path = Field_SearchBreadthFirst(field, start, goal);
    const int32_t t1 = Util_Time();

    field = Field_Trace(field, path);
    Field_Print(field);

    Queue_Free(path);
    Field_Free(field);
    printf("Path finding took %d us (microseconds)\n", t1 - t0);
}
