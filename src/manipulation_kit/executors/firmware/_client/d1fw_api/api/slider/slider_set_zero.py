from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.slider_set_zero_request import SliderSetZeroRequest
from ...models.slider_set_zero_response_200 import SliderSetZeroResponse200
from ...types import Response


def _get_kwargs(
    *,
    body: SliderSetZeroRequest,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/slider/set_zero",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | SliderSetZeroResponse200 | None:
    if response.status_code == 200:
        response_200 = SliderSetZeroResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | SliderSetZeroResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: SliderSetZeroRequest,
) -> Response[ErrorEnvelope | SliderSetZeroResponse200]:
    """COMMISSIONING ONLY: rewrite the torso lift's origin to where it stands now

     **This is not a motion verb and does not belong on an operator console.** It throws the drive's
    current zero away and makes the carriage's present position the new 0 m, so every height anything
    was calibrated against moves with it. Perform it with the lift parked on the mechanical bottom plus
    3-4 mm, once, when the drive has no usable zero — a new drive, a replaced motor, an EEPROM that did
    not survive — which is the state `slider/state` reports as `zero_reference: "unknown"` and which
    makes `set_height` and `home` answer `409`.

    On the wire it is the LD2-RS commissioning sequence: `Pr0.15 = 9` (clear the absolute encoder's
    multi-turn count, after which the drive restores the parameter to `1` itself), PR trigger `0x0021`
    (clear the in-RAM position counter), then `0x1801 = 0x2211` (save parameters) polled at `0x1901`
    until `0x5555`. A save that reports `0xAAAA` is a `502`: the origin would live in RAM only and be
    gone at the next power cycle.

    Two gates, both required. The daemon must have been started with `[slider] commissioning = true`
    (otherwise `409`, with nothing sent to the drive), and the body must carry `{"confirm": "SET_ZERO"}`
    (otherwise `400`). The slider's soft-kill latch applies, and a lift that is moving is refused.
    Afterwards, power-cycle the robot and check `slider/state` again. That step is now mechanical: a
    successful `set_zero` records the new origin as `zero_verification: "pending"` in the daemon's state
    file, and the next start promotes it to `"verified"` by itself when the height still reads what it
    read here. Leave the carriage where `set_zero` left it until that power cycle; a lift that was moved
    in between reads something else and stays `pending`, which one more power cycle clears. An origin
    that did not persist is the failure this verb exists to prevent, and `pending` is the daemon saying
    so out loud.

    Args:
        body (SliderSetZeroRequest): `POST /v1/slider/set_zero`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderSetZeroResponse200]
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
    body: SliderSetZeroRequest,
) -> ErrorEnvelope | SliderSetZeroResponse200 | None:
    """COMMISSIONING ONLY: rewrite the torso lift's origin to where it stands now

     **This is not a motion verb and does not belong on an operator console.** It throws the drive's
    current zero away and makes the carriage's present position the new 0 m, so every height anything
    was calibrated against moves with it. Perform it with the lift parked on the mechanical bottom plus
    3-4 mm, once, when the drive has no usable zero — a new drive, a replaced motor, an EEPROM that did
    not survive — which is the state `slider/state` reports as `zero_reference: "unknown"` and which
    makes `set_height` and `home` answer `409`.

    On the wire it is the LD2-RS commissioning sequence: `Pr0.15 = 9` (clear the absolute encoder's
    multi-turn count, after which the drive restores the parameter to `1` itself), PR trigger `0x0021`
    (clear the in-RAM position counter), then `0x1801 = 0x2211` (save parameters) polled at `0x1901`
    until `0x5555`. A save that reports `0xAAAA` is a `502`: the origin would live in RAM only and be
    gone at the next power cycle.

    Two gates, both required. The daemon must have been started with `[slider] commissioning = true`
    (otherwise `409`, with nothing sent to the drive), and the body must carry `{"confirm": "SET_ZERO"}`
    (otherwise `400`). The slider's soft-kill latch applies, and a lift that is moving is refused.
    Afterwards, power-cycle the robot and check `slider/state` again. That step is now mechanical: a
    successful `set_zero` records the new origin as `zero_verification: "pending"` in the daemon's state
    file, and the next start promotes it to `"verified"` by itself when the height still reads what it
    read here. Leave the carriage where `set_zero` left it until that power cycle; a lift that was moved
    in between reads something else and stays `pending`, which one more power cycle clears. An origin
    that did not persist is the failure this verb exists to prevent, and `pending` is the daemon saying
    so out loud.

    Args:
        body (SliderSetZeroRequest): `POST /v1/slider/set_zero`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderSetZeroResponse200
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: SliderSetZeroRequest,
) -> Response[ErrorEnvelope | SliderSetZeroResponse200]:
    """COMMISSIONING ONLY: rewrite the torso lift's origin to where it stands now

     **This is not a motion verb and does not belong on an operator console.** It throws the drive's
    current zero away and makes the carriage's present position the new 0 m, so every height anything
    was calibrated against moves with it. Perform it with the lift parked on the mechanical bottom plus
    3-4 mm, once, when the drive has no usable zero — a new drive, a replaced motor, an EEPROM that did
    not survive — which is the state `slider/state` reports as `zero_reference: "unknown"` and which
    makes `set_height` and `home` answer `409`.

    On the wire it is the LD2-RS commissioning sequence: `Pr0.15 = 9` (clear the absolute encoder's
    multi-turn count, after which the drive restores the parameter to `1` itself), PR trigger `0x0021`
    (clear the in-RAM position counter), then `0x1801 = 0x2211` (save parameters) polled at `0x1901`
    until `0x5555`. A save that reports `0xAAAA` is a `502`: the origin would live in RAM only and be
    gone at the next power cycle.

    Two gates, both required. The daemon must have been started with `[slider] commissioning = true`
    (otherwise `409`, with nothing sent to the drive), and the body must carry `{"confirm": "SET_ZERO"}`
    (otherwise `400`). The slider's soft-kill latch applies, and a lift that is moving is refused.
    Afterwards, power-cycle the robot and check `slider/state` again. That step is now mechanical: a
    successful `set_zero` records the new origin as `zero_verification: "pending"` in the daemon's state
    file, and the next start promotes it to `"verified"` by itself when the height still reads what it
    read here. Leave the carriage where `set_zero` left it until that power cycle; a lift that was moved
    in between reads something else and stays `pending`, which one more power cycle clears. An origin
    that did not persist is the failure this verb exists to prevent, and `pending` is the daemon saying
    so out loud.

    Args:
        body (SliderSetZeroRequest): `POST /v1/slider/set_zero`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | SliderSetZeroResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: SliderSetZeroRequest,
) -> ErrorEnvelope | SliderSetZeroResponse200 | None:
    """COMMISSIONING ONLY: rewrite the torso lift's origin to where it stands now

     **This is not a motion verb and does not belong on an operator console.** It throws the drive's
    current zero away and makes the carriage's present position the new 0 m, so every height anything
    was calibrated against moves with it. Perform it with the lift parked on the mechanical bottom plus
    3-4 mm, once, when the drive has no usable zero — a new drive, a replaced motor, an EEPROM that did
    not survive — which is the state `slider/state` reports as `zero_reference: "unknown"` and which
    makes `set_height` and `home` answer `409`.

    On the wire it is the LD2-RS commissioning sequence: `Pr0.15 = 9` (clear the absolute encoder's
    multi-turn count, after which the drive restores the parameter to `1` itself), PR trigger `0x0021`
    (clear the in-RAM position counter), then `0x1801 = 0x2211` (save parameters) polled at `0x1901`
    until `0x5555`. A save that reports `0xAAAA` is a `502`: the origin would live in RAM only and be
    gone at the next power cycle.

    Two gates, both required. The daemon must have been started with `[slider] commissioning = true`
    (otherwise `409`, with nothing sent to the drive), and the body must carry `{"confirm": "SET_ZERO"}`
    (otherwise `400`). The slider's soft-kill latch applies, and a lift that is moving is refused.
    Afterwards, power-cycle the robot and check `slider/state` again. That step is now mechanical: a
    successful `set_zero` records the new origin as `zero_verification: "pending"` in the daemon's state
    file, and the next start promotes it to `"verified"` by itself when the height still reads what it
    read here. Leave the carriage where `set_zero` left it until that power cycle; a lift that was moved
    in between reads something else and stays `pending`, which one more power cycle clears. An origin
    that did not persist is the failure this verb exists to prevent, and `pending` is the daemon saying
    so out loud.

    Args:
        body (SliderSetZeroRequest): `POST /v1/slider/set_zero`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | SliderSetZeroResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
