from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.slider_move import SliderMove
from ...models.slider_set_height_response_200 import SliderSetHeightResponse200
from ...types import Response


def _get_kwargs(
    *,
    body: SliderMove,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/slider/set_height",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | SliderSetHeightResponse200 | None:
    if response.status_code == 200:
        response_200 = SliderSetHeightResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope | SliderSetHeightResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: SliderMove,
) -> Response[ErrorEnvelope | SliderSetHeightResponse200]:
    """Move the torso lift to an absolute height, or by one small commissioning step

     Exactly one of `height_m` and `delta_m` is sent; neither and both are `400`.

    `height_m` is metres above the drive's origin. With `wait: true` the call blocks until the slider
    reports arrival, which is the slowest call in this API; size the client timeout accordingly. It is
    refused with `409` while `slider/state` reports `zero_reference: "unknown"`: the height is measured
    from the drive's origin, and a drive that cannot vouch for its origin would take this move somewhere
    other than where it names. The refusal carries an `advisory` naming the way out. It is refused with
    `400` when it falls outside the effective range in `slider/state.travel_limits`, and the message
    names WHICH layer refused it -- the model ceiling `[slider] travel_max_m`, or this unit's recorded
    calibration.

    `delta_m` is a small RELATIVE amount in metres from wherever the carriage stands, negative for down.
    It depends on no origin, so it is the one move a lift with `zero_reference: "unknown"` can be given
    -- which is how a carriage is walked down onto its mechanical stop before `slider/set_zero`. It is
    admitted only on a daemon started with `[slider] commissioning = true` (otherwise `409` with an
    `advisory`), capped at 0.01 m per call (`400` above that, never silently clamped), and always runs
    at a fixed low speed, so sending `speed_ratio` with it is a `400` rather than a value quietly
    ignored. The slider soft-kill latch and `slider/stop` apply to it exactly as to any move.

    Args:
        body (SliderMove): A requested slider move: one absolute height, or one small relative
            amount.

            Exactly one of `height_m` and `delta_m` is given. Neither and both are
            both rejected with `400` rather than resolved by a precedence rule: a
            caller that sent two targets did not mean one of them, and a caller that
            sent none named no move at all.

            `delta_m` is the commissioning form. It is admitted only on a daemon
            started with `[slider] commissioning = true`, it is capped at
            [`crate::safety::SLIDER_COMMISSIONING_STEP_MAX_M`] per call, and it runs
            at a fixed low speed — so `speed_ratio` may not be sent with it. It is on
            this verb rather than on a new one because on the LD2-RS's wire the two
            are the same PR path write with a different mode word (`0x0041` instead
            of `0x0001`), so a separate verb would have been a second name for one
            operation.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderSetHeightResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: SliderMove,
) -> ErrorEnvelope | SliderSetHeightResponse200 | None:
    """Move the torso lift to an absolute height, or by one small commissioning step

     Exactly one of `height_m` and `delta_m` is sent; neither and both are `400`.

    `height_m` is metres above the drive's origin. With `wait: true` the call blocks until the slider
    reports arrival, which is the slowest call in this API; size the client timeout accordingly. It is
    refused with `409` while `slider/state` reports `zero_reference: "unknown"`: the height is measured
    from the drive's origin, and a drive that cannot vouch for its origin would take this move somewhere
    other than where it names. The refusal carries an `advisory` naming the way out. It is refused with
    `400` when it falls outside the effective range in `slider/state.travel_limits`, and the message
    names WHICH layer refused it -- the model ceiling `[slider] travel_max_m`, or this unit's recorded
    calibration.

    `delta_m` is a small RELATIVE amount in metres from wherever the carriage stands, negative for down.
    It depends on no origin, so it is the one move a lift with `zero_reference: "unknown"` can be given
    -- which is how a carriage is walked down onto its mechanical stop before `slider/set_zero`. It is
    admitted only on a daemon started with `[slider] commissioning = true` (otherwise `409` with an
    `advisory`), capped at 0.01 m per call (`400` above that, never silently clamped), and always runs
    at a fixed low speed, so sending `speed_ratio` with it is a `400` rather than a value quietly
    ignored. The slider soft-kill latch and `slider/stop` apply to it exactly as to any move.

    Args:
        body (SliderMove): A requested slider move: one absolute height, or one small relative
            amount.

            Exactly one of `height_m` and `delta_m` is given. Neither and both are
            both rejected with `400` rather than resolved by a precedence rule: a
            caller that sent two targets did not mean one of them, and a caller that
            sent none named no move at all.

            `delta_m` is the commissioning form. It is admitted only on a daemon
            started with `[slider] commissioning = true`, it is capped at
            [`crate::safety::SLIDER_COMMISSIONING_STEP_MAX_M`] per call, and it runs
            at a fixed low speed — so `speed_ratio` may not be sent with it. It is on
            this verb rather than on a new one because on the LD2-RS's wire the two
            are the same PR path write with a different mode word (`0x0041` instead
            of `0x0001`), so a separate verb would have been a second name for one
            operation.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderSetHeightResponse200
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: SliderMove,
) -> Response[ErrorEnvelope | SliderSetHeightResponse200]:
    """Move the torso lift to an absolute height, or by one small commissioning step

     Exactly one of `height_m` and `delta_m` is sent; neither and both are `400`.

    `height_m` is metres above the drive's origin. With `wait: true` the call blocks until the slider
    reports arrival, which is the slowest call in this API; size the client timeout accordingly. It is
    refused with `409` while `slider/state` reports `zero_reference: "unknown"`: the height is measured
    from the drive's origin, and a drive that cannot vouch for its origin would take this move somewhere
    other than where it names. The refusal carries an `advisory` naming the way out. It is refused with
    `400` when it falls outside the effective range in `slider/state.travel_limits`, and the message
    names WHICH layer refused it -- the model ceiling `[slider] travel_max_m`, or this unit's recorded
    calibration.

    `delta_m` is a small RELATIVE amount in metres from wherever the carriage stands, negative for down.
    It depends on no origin, so it is the one move a lift with `zero_reference: "unknown"` can be given
    -- which is how a carriage is walked down onto its mechanical stop before `slider/set_zero`. It is
    admitted only on a daemon started with `[slider] commissioning = true` (otherwise `409` with an
    `advisory`), capped at 0.01 m per call (`400` above that, never silently clamped), and always runs
    at a fixed low speed, so sending `speed_ratio` with it is a `400` rather than a value quietly
    ignored. The slider soft-kill latch and `slider/stop` apply to it exactly as to any move.

    Args:
        body (SliderMove): A requested slider move: one absolute height, or one small relative
            amount.

            Exactly one of `height_m` and `delta_m` is given. Neither and both are
            both rejected with `400` rather than resolved by a precedence rule: a
            caller that sent two targets did not mean one of them, and a caller that
            sent none named no move at all.

            `delta_m` is the commissioning form. It is admitted only on a daemon
            started with `[slider] commissioning = true`, it is capped at
            [`crate::safety::SLIDER_COMMISSIONING_STEP_MAX_M`] per call, and it runs
            at a fixed low speed — so `speed_ratio` may not be sent with it. It is on
            this verb rather than on a new one because on the LD2-RS's wire the two
            are the same PR path write with a different mode word (`0x0041` instead
            of `0x0001`), so a separate verb would have been a second name for one
            operation.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderSetHeightResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: SliderMove,
) -> ErrorEnvelope | SliderSetHeightResponse200 | None:
    """Move the torso lift to an absolute height, or by one small commissioning step

     Exactly one of `height_m` and `delta_m` is sent; neither and both are `400`.

    `height_m` is metres above the drive's origin. With `wait: true` the call blocks until the slider
    reports arrival, which is the slowest call in this API; size the client timeout accordingly. It is
    refused with `409` while `slider/state` reports `zero_reference: "unknown"`: the height is measured
    from the drive's origin, and a drive that cannot vouch for its origin would take this move somewhere
    other than where it names. The refusal carries an `advisory` naming the way out. It is refused with
    `400` when it falls outside the effective range in `slider/state.travel_limits`, and the message
    names WHICH layer refused it -- the model ceiling `[slider] travel_max_m`, or this unit's recorded
    calibration.

    `delta_m` is a small RELATIVE amount in metres from wherever the carriage stands, negative for down.
    It depends on no origin, so it is the one move a lift with `zero_reference: "unknown"` can be given
    -- which is how a carriage is walked down onto its mechanical stop before `slider/set_zero`. It is
    admitted only on a daemon started with `[slider] commissioning = true` (otherwise `409` with an
    `advisory`), capped at 0.01 m per call (`400` above that, never silently clamped), and always runs
    at a fixed low speed, so sending `speed_ratio` with it is a `400` rather than a value quietly
    ignored. The slider soft-kill latch and `slider/stop` apply to it exactly as to any move.

    Args:
        body (SliderMove): A requested slider move: one absolute height, or one small relative
            amount.

            Exactly one of `height_m` and `delta_m` is given. Neither and both are
            both rejected with `400` rather than resolved by a precedence rule: a
            caller that sent two targets did not mean one of them, and a caller that
            sent none named no move at all.

            `delta_m` is the commissioning form. It is admitted only on a daemon
            started with `[slider] commissioning = true`, it is capped at
            [`crate::safety::SLIDER_COMMISSIONING_STEP_MAX_M`] per call, and it runs
            at a fixed low speed — so `speed_ratio` may not be sent with it. It is on
            this verb rather than on a new one because on the LD2-RS's wire the two
            are the same PR path write with a different mode word (`0x0041` instead
            of `0x0001`), so a separate verb would have been a second name for one
            operation.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderSetHeightResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
