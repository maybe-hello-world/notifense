#include "notification_processor.h"

#include "haptic_manager.h"

namespace notificationProcessor {
namespace {

constexpr uint8_t DEFAULT_NOTIFICATION_EFFECT = 1;

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

} // namespace

void decideNotificationAction(const NotificationDetails &notification)
{
    // Notification-specific haptic selection will be added here later.
    (void) notification;
    if (notification.preExisting || notification.silent) {
        return;
    }
    hapticManager::playEffect(DEFAULT_NOTIFICATION_EFFECT);
}

// Internal parsing, printing, and dispatch code below is not intended for
// modification when changing notification-to-haptic behavior.

namespace {

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

void loadNotificationAttributes(
    BLEAncs &ancsClient,
    NotificationDetails &notification
)
{
    notification.appIdLength = ancsClient.getAppID(
        notification.uid,
        notification.appId,
        sizeof(notification.appId) - 1
    );

    if (notification.appIdLength > 0) {
        notification.appNameLength = ancsClient.getAppAttribute(
            notification.appId,
            ANCS_APP_ATTR_DISPLAY_NAME,
            notification.appName,
            sizeof(notification.appName) - 1
        );
    }

    notification.titleLength = ancsClient.getTitle(
        notification.uid,
        notification.title,
        sizeof(notification.title) - 1
    );
    notification.subtitleLength = ancsClient.getSubtitle(
        notification.uid,
        notification.subtitle,
        sizeof(notification.subtitle) - 1
    );
    notification.messageLength = ancsClient.getMessage(
        notification.uid,
        notification.message,
        sizeof(notification.message) - 1
    );
    notification.dateLength = ancsClient.getDate(
        notification.uid,
        notification.date,
        sizeof(notification.date) - 1
    );
}

void printAttribute(
    const __FlashStringHelper *label,
    const char *value,
    uint16_t length
)
{
    Serial.print(label);
    Serial.println(length > 0 ? value : "<unavailable>");
}

void printNotificationDetails(const NotificationDetails &notification)
{
    Serial.println();
    Serial.println(F("========== iOS notification =========="));
    Serial.print(F("[ANCS] Event: "));
    Serial.print(eventName(notification.eventId));
    Serial.print(F(" ("));
    Serial.print(notification.eventId);
    Serial.println(')');

    Serial.print(F("[ANCS] Category: "));
    Serial.print(categoryName(notification.categoryId));
    Serial.print(F(" ("));
    Serial.print(notification.categoryId);
    Serial.println(')');

    Serial.print(F("[ANCS] Category count: "));
    Serial.println(notification.categoryCount);
    Serial.print(F("[ANCS] UID: "));
    Serial.println(notification.uid);

    Serial.print(F("[ANCS] Flags: silent="));
    Serial.print(notification.silent);
    Serial.print(F(", important="));
    Serial.print(notification.important);
    Serial.print(F(", pre-existing="));
    Serial.print(notification.preExisting);
    Serial.print(F(", positive-action="));
    Serial.print(notification.positiveAction);
    Serial.print(F(", negative-action="));
    Serial.println(notification.negativeAction);

    if (notification.eventId != ANCS_EVT_NOTIFICATION_REMOVED) {
        printAttribute(F("[ANCS] App ID: "), notification.appId, notification.appIdLength);
        printAttribute(F("[ANCS] App name: "), notification.appName, notification.appNameLength);
        printAttribute(F("[ANCS] Title: "), notification.title, notification.titleLength);
        printAttribute(F("[ANCS] Subtitle: "), notification.subtitle, notification.subtitleLength);
        printAttribute(F("[ANCS] Message: "), notification.message, notification.messageLength);
        printAttribute(F("[ANCS] Date: "), notification.date, notification.dateLength);
    } else {
        Serial.println(F("[ANCS] Removed notification; attributes are no longer requested"));
    }

    Serial.println(F("======================================"));
}

} // namespace

void processNotification(
    BLEAncs &ancsClient,
    const AncsNotification_t *notification
)
{
    if (notification == nullptr) {
        return;
    }

    NotificationDetails details = {};
    details.eventId = notification->eventID;
    details.categoryId = notification->categoryID;
    details.categoryCount = notification->categoryCount;
    details.uid = notification->uid;
    details.silent = notification->eventFlags.silent;
    details.important = notification->eventFlags.important;
    details.preExisting = notification->eventFlags.preExisting;
    details.positiveAction = notification->eventFlags.positiveAction;
    details.negativeAction = notification->eventFlags.NegativeAction;

    if (details.eventId != ANCS_EVT_NOTIFICATION_REMOVED) {
        loadNotificationAttributes(ancsClient, details);
    }

    printNotificationDetails(details);
    decideNotificationAction(details);
}

} // namespace notificationProcessor
