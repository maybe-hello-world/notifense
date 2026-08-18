#include "notification_processor.h"

#include "haptic_manager.h"

namespace notificationProcessor {
namespace {

constexpr uint8_t DEFAULT_NOTIFICATION_EFFECT = 13;
constexpr uint8_t IMPORTANT_NOTIFICATION_EFFECT = 67;

struct AppHapticRule {
    const char *appId;
    uint8_t effectId;
    uint8_t count;
};

struct CategoryHapticRule {
    uint8_t categoryId;
    uint8_t effectId;
    uint8_t count;
};

// App-specific haptic rules. 
constexpr AppHapticRule APP_HAPTIC_RULES[] = {
    {"com.google.Gmail", 37, 2},
    {"com.google.Calendar", 48, 2},

    {"com.openai.chat", 1, 1},

    {"com.hammerandchisel.discord", 27, 1},
    {"ph.telegra.Telegraph", 26, 3},
};


// Sparse category mapping.
// ANCS category IDs correspond to your CATEGORY_NAMES table:
// 0 Other
// 1 Incoming Call
// 2 Missed Call
// 3 Voice Mail
// 4 Social
// 5 Schedule
// 6 Email
// 7 News
// 8 Health/Fitness
// 9 Business/Finance
// 10 Location
// 11 Entertainment
constexpr CategoryHapticRule CATEGORY_HAPTIC_RULES[] = {
    {1, 15, 1},  // Incoming Call
    {2, 3, 5},  // Missed Call
};

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

void playEffect(uint8_t effectId, uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) {
        hapticManager::playEffect(effectId);
    }
}


const AppHapticRule *findAppRule(const NotificationDetails &notification)
{
    if (notification.appIdLength == 0) {
        return nullptr;
    }

    for (const auto &rule : APP_HAPTIC_RULES) {
        const size_t ruleLength = strlen(rule.appId);

        if (
            notification.appIdLength == ruleLength
            && memcmp(notification.appId, rule.appId, ruleLength) == 0
        ) {
            return &rule;
        }
    }

    return nullptr;
}


const CategoryHapticRule *findCategoryRule(uint8_t categoryId)
{
    for (const auto &rule : CATEGORY_HAPTIC_RULES) {
        if (rule.categoryId == categoryId) {
            return &rule;
        }
    }

    return nullptr;
}

} // namespace

void decideNotificationAction(const NotificationDetails &notification)
{
    // Ignore silent or pre-existing notifications.
    if (notification.preExisting || notification.silent) {
        return;
    }

    // Only react to newly-added notifications.
    if (notification.eventId != ANCS_EVT_NOTIFICATION_ADDED) {
        return;
    }

    // Important notifications get a pre-notification haptic.
    if (notification.important) {
        hapticManager::playEffect(IMPORTANT_NOTIFICATION_EFFECT);
    }

    // 1. App-specific rule has highest priority.
    if (const AppHapticRule *rule = findAppRule(notification)) {
        playEffect(rule->effectId, rule->count);
        return;
    }

    // 2. Otherwise try category-specific rule.
    if (const CategoryHapticRule *rule =
            findCategoryRule(notification.categoryId)) {
        playEffect(rule->effectId, rule->count);
        return;
    }

    // 3. Otherwise default notification effect.
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
