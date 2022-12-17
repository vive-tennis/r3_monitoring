#ifndef TOPIC_H
#define TOPIC_H
#include <QString>

struct Topic {
    QString name;
    QString type;
    QString time_created;
    bool  excluded;
};

#endif // TOPIC_H
