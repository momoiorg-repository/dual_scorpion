"""Operator mounting declaration; actual sensor reassignment happens in Sony app."""


def mounting_prompts(cfg, camera=True):
    mode = cfg["mocopi"]["tracking_mode"]
    prompts = [
        "HEADを頭に装着し、C270を同じHead bandへ固定してください" if camera else "HEADを頭に装着してください",
        "WRIST/Lを左手首に装着してください",
        "WRIST/Rを右手首に装着してください",
    ]
    if mode == "upper_body":
        prompts.extend(
            [
                "スマホで『上半身集中』を選択し、ANKLE/Lを左二の腕へ、アプリ案内の位置・向きで装着してください",
                "ANKLE/Rを右二の腕へ、アプリ案内の位置・向きで装着してください",
            ]
        )
    else:
        prompts.extend(["ANKLE/Lを左足首に装着してください", "ANKLE/Rを右足首に装着してください"])
    prompts.append("HIPを腰へ固定し、選択したモードでスマホ側のcalibrationを完了してください")
    return prompts


def mounting_summary(cfg, camera=True):
    mode = cfg["mocopi"]["tracking_mode"]
    return (
        f"Declared mocopi mode: {mode} (アプリ側で同じモードを選択してください。UDPから自動検出しません)\n"
        + "\n".join(f"{i}. {text}" for i, text in enumerate(mounting_prompts(cfg, camera=camera), 1))
    )
