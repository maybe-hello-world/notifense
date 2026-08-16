#pragma once

#include <Arduino.h>
#include <bluefruit.h>

namespace notificationProcessor {

constexpr uint8_t ATTRIBUTE_BUFFER_SIZE = 128;

struct NotificationDetails {
    uint8_t eventId;
    uint8_t categoryId;
    uint8_t categoryCount;
    uint32_t uid;

    bool silent;
    bool important;
    bool preExisting;
    bool positiveAction;
    bool negativeAction;

    char appId[ATTRIBUTE_BUFFER_SIZE];
    char appName[ATTRIBUTE_BUFFER_SIZE];
    char title[ATTRIBUTE_BUFFER_SIZE];
    char subtitle[ATTRIBUTE_BUFFER_SIZE];
    char message[ATTRIBUTE_BUFFER_SIZE];
    char date[ATTRIBUTE_BUFFER_SIZE];

    uint16_t appIdLength;
    uint16_t appNameLength;
    uint16_t titleLength;
    uint16_t subtitleLength;
    uint16_t messageLength;
    uint16_t dateLength;
};

void decideNotificationAction(const NotificationDetails &notification);
void processNotification(
    BLEAncs &ancsClient,
    const AncsNotification_t *notification
);

} // namespace notificationProcessor
