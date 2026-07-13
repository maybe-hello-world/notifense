#pragma once

#include <Arduino.h>
#include <bluefruit.h>

namespace notificationProcessor {

constexpr uint8_t ATTRIBUTE_BUFFER_SIZE = 128;

char attributeBuffer[ATTRIBUTE_BUFFER_SIZE];

const char *const EVENT_NAMES[] = {
    "Added",
    "Modified",
    "Removed"
};

const char *const CATEGORY_NAMES[] = {
    "Other",
    "Incoming Call",
    "Missed Call",
    "Voice Mail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "Health and Fitness",
    "Business and Finance",
    "Location",
    "Entertainment"
};

const char *eventName(uint8_t eventId)
{
    if (eventId < (sizeof(EVENT_NAMES) / sizeof(EVENT_NAMES[0]))) {
        return EVENT_NAMES[eventId];
    }
    return "Unknown";
}

const char *categoryName(uint8_t categoryId)
{
    if (categoryId < (sizeof(CATEGORY_NAMES) / sizeof(CATEGORY_NAMES[0]))) {
        return CATEGORY_NAMES[categoryId];
    }
    return "Unknown";
}

void printAttribute(const __FlashStringHelper *label, uint16_t length)
{
    Serial.print(label);
    if (length > 0) {
        Serial.println(attributeBuffer);
    } else {
        Serial.println(F("<unavailable>"));
    }
}

void printNotificationAttributes(BLEAncs &ancsClient, uint32_t notificationUid)
{
    char appId[ATTRIBUTE_BUFFER_SIZE] = {0};
    const uint16_t appIdLength = ancsClient.getAppID(
        notificationUid,
        appId,
        sizeof(appId) - 1
    );

    Serial.print(F("[ANCS] App ID: "));
    Serial.println(appIdLength > 0 ? appId : "<unavailable>");

    if (appIdLength > 0) {
        memset(attributeBuffer, 0, sizeof(attributeBuffer));
        const uint16_t appNameLength = ancsClient.getAppAttribute(
            appId,
            ANCS_APP_ATTR_DISPLAY_NAME,
            attributeBuffer,
            sizeof(attributeBuffer) - 1
        );
        printAttribute(F("[ANCS] App name: "), appNameLength);
    }

    memset(attributeBuffer, 0, sizeof(attributeBuffer));
    printAttribute(
        F("[ANCS] Title: "),
        ancsClient.getTitle(
            notificationUid,
            attributeBuffer,
            sizeof(attributeBuffer) - 1
        )
    );

    memset(attributeBuffer, 0, sizeof(attributeBuffer));
    printAttribute(
        F("[ANCS] Subtitle: "),
        ancsClient.getSubtitle(
            notificationUid,
            attributeBuffer,
            sizeof(attributeBuffer) - 1
        )
    );

    memset(attributeBuffer, 0, sizeof(attributeBuffer));
    printAttribute(
        F("[ANCS] Message: "),
        ancsClient.getMessage(
            notificationUid,
            attributeBuffer,
            sizeof(attributeBuffer) - 1
        )
    );

    memset(attributeBuffer, 0, sizeof(attributeBuffer));
    printAttribute(
        F("[ANCS] Date: "),
        ancsClient.getDate(
            notificationUid,
            attributeBuffer,
            sizeof(attributeBuffer) - 1
        )
    );
}

void processNotification(
    BLEAncs &ancsClient,
    const AncsNotification_t *notification
)
{
    if (notification == nullptr) {
        return;
    }

    Serial.println();
    Serial.println(F("========== iOS notification =========="));
    Serial.print(F("[ANCS] Event: "));
    Serial.print(eventName(notification->eventID));
    Serial.print(F(" ("));
    Serial.print(notification->eventID);
    Serial.println(')');

    Serial.print(F("[ANCS] Category: "));
    Serial.print(categoryName(notification->categoryID));
    Serial.print(F(" ("));
    Serial.print(notification->categoryID);
    Serial.println(')');

    Serial.print(F("[ANCS] Category count: "));
    Serial.println(notification->categoryCount);
    Serial.print(F("[ANCS] UID: "));
    Serial.println(notification->uid);

    Serial.print(F("[ANCS] Flags: silent="));
    Serial.print(notification->eventFlags.silent);
    Serial.print(F(", important="));
    Serial.print(notification->eventFlags.important);
    Serial.print(F(", pre-existing="));
    Serial.print(notification->eventFlags.preExisting);
    Serial.print(F(", positive-action="));
    Serial.print(notification->eventFlags.positiveAction);
    Serial.print(F(", negative-action="));
    Serial.println(notification->eventFlags.NegativeAction);

    if (notification->eventID != ANCS_EVT_NOTIFICATION_REMOVED) {
        printNotificationAttributes(ancsClient, notification->uid);
    } else {
        Serial.println(F("[ANCS] Removed notification; attributes are no longer requested"));
    }

    Serial.println(F("======================================"));
}

} // namespace notificationProcessor
