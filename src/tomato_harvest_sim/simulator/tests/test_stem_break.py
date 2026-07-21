from tomato_harvest_sim.simulator.stem_break import (
    StemBreakDecision,
    StemBreakEventMatcher,
    encoded_joint_path_parts,
)


def test_encoded_joint_path_parts_accepts_item_only_payload() -> None:
    class ItemOnlyPayload:
        def __getitem__(self, key: str) -> tuple[int, int]:
            assert key == "jointPath"
            return (12, 34)

    assert encoded_joint_path_parts(ItemOnlyPayload()) == (12, 34)


def test_target_joint_break_is_accepted_once_per_cycle() -> None:
    matcher = StemBreakEventMatcher("/World/TomatoStemJoint")

    first = matcher.observe("joint_break", "/World/TomatoStemJoint")
    duplicate = matcher.observe("joint_break", "/World/TomatoStemJoint")

    assert first is StemBreakDecision.TARGET_BROKEN
    assert duplicate is StemBreakDecision.DUPLICATE


def test_unrelated_events_are_ignored() -> None:
    matcher = StemBreakEventMatcher("/World/TomatoStemJoint")

    assert (
        matcher.observe("contact_found", "/World/TomatoStemJoint")
        is StemBreakDecision.IGNORED_EVENT_TYPE
    )
    assert (
        matcher.observe("joint_break", "/World/OtherJoint")
        is StemBreakDecision.IGNORED_JOINT
    )
    assert (
        matcher.observe("joint_break", None)
        is StemBreakDecision.INVALID_JOINT_PATH
    )


def test_reset_accepts_the_next_cycle_break() -> None:
    matcher = StemBreakEventMatcher("/World/TomatoStemJoint")
    matcher.observe("joint_break", "/World/TomatoStemJoint")

    matcher.reset()

    assert (
        matcher.observe("joint_break", "/World/TomatoStemJoint")
        is StemBreakDecision.TARGET_BROKEN
    )
