from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.slider_home_response_200 import SliderHomeResponse200
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/slider/home",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | SliderHomeResponse200 | None:
    if response.status_code == 200:
        response_200 = SliderHomeResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | SliderHomeResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | SliderHomeResponse200]:
    """Move the torso lift to 0 m of the current origin

     Move the torso lift to 0 m of the current origin (an absolute move, not the drive's homing routine).
    It is exactly `slider/set_height` with `height_m` 0 and the default speed, and like an unwaited move
    it returns as soon as the drive has accepted the target — poll `GET /v1/slider/state` for arrival.

    This verb does NOT re-reference the drive. Homing, which seeks the mechanical stop and re-zeroes the
    position counter there, moves whatever origin the robot was commissioned with. That procedure is
    `slider/set_zero`, which is gated off by default and is not reachable from this verb.

    Refused with `409` while `slider/state` reports `zero_reference: "unknown"` — a move to 0 m of an
    origin the drive cannot vouch for is exactly how the lift reaches its mechanical stop. It is refused
    the same way when a recorded unit calibration puts its lower bound above 0 m: on such a robot there
    is no 0 m to go to, and the refusal names that layer.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderHomeResponse200]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | SliderHomeResponse200 | None:
    """Move the torso lift to 0 m of the current origin

     Move the torso lift to 0 m of the current origin (an absolute move, not the drive's homing routine).
    It is exactly `slider/set_height` with `height_m` 0 and the default speed, and like an unwaited move
    it returns as soon as the drive has accepted the target — poll `GET /v1/slider/state` for arrival.

    This verb does NOT re-reference the drive. Homing, which seeks the mechanical stop and re-zeroes the
    position counter there, moves whatever origin the robot was commissioned with. That procedure is
    `slider/set_zero`, which is gated off by default and is not reachable from this verb.

    Refused with `409` while `slider/state` reports `zero_reference: "unknown"` — a move to 0 m of an
    origin the drive cannot vouch for is exactly how the lift reaches its mechanical stop. It is refused
    the same way when a recorded unit calibration puts its lower bound above 0 m: on such a robot there
    is no 0 m to go to, and the refusal names that layer.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderHomeResponse200
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | SliderHomeResponse200]:
    """Move the torso lift to 0 m of the current origin

     Move the torso lift to 0 m of the current origin (an absolute move, not the drive's homing routine).
    It is exactly `slider/set_height` with `height_m` 0 and the default speed, and like an unwaited move
    it returns as soon as the drive has accepted the target — poll `GET /v1/slider/state` for arrival.

    This verb does NOT re-reference the drive. Homing, which seeks the mechanical stop and re-zeroes the
    position counter there, moves whatever origin the robot was commissioned with. That procedure is
    `slider/set_zero`, which is gated off by default and is not reachable from this verb.

    Refused with `409` while `slider/state` reports `zero_reference: "unknown"` — a move to 0 m of an
    origin the drive cannot vouch for is exactly how the lift reaches its mechanical stop. It is refused
    the same way when a recorded unit calibration puts its lower bound above 0 m: on such a robot there
    is no 0 m to go to, and the refusal names that layer.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderHomeResponse200]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | SliderHomeResponse200 | None:
    """Move the torso lift to 0 m of the current origin

     Move the torso lift to 0 m of the current origin (an absolute move, not the drive's homing routine).
    It is exactly `slider/set_height` with `height_m` 0 and the default speed, and like an unwaited move
    it returns as soon as the drive has accepted the target — poll `GET /v1/slider/state` for arrival.

    This verb does NOT re-reference the drive. Homing, which seeks the mechanical stop and re-zeroes the
    position counter there, moves whatever origin the robot was commissioned with. That procedure is
    `slider/set_zero`, which is gated off by default and is not reachable from this verb.

    Refused with `409` while `slider/state` reports `zero_reference: "unknown"` — a move to 0 m of an
    origin the drive cannot vouch for is exactly how the lift reaches its mechanical stop. It is refused
    the same way when a recorded unit calibration puts its lower bound above 0 m: on such a robot there
    is no 0 m to go to, and the refusal names that layer.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderHomeResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
