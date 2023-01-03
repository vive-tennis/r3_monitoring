#ifndef TOPIC_H
#define TOPIC_H
#include <QString>

struct TopicName {
    QString name;
    QString type;
    QString time_created;
    bool included;
};

struct TopicType {
    QString name;
    bool included;
};

#endif // TOPIC_H
