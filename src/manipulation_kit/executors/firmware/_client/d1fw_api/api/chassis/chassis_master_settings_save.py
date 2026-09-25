from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_master_settings_save_response_200 import (
    ChassisMasterSettingsSaveResponse200,
)
from ...models.chassis_settings_save import ChassisSettingsSave
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ChassisSettingsSave,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/master_settings",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisMasterSettingsSaveResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisMasterSettingsSaveResponse200.from_dict(response.json())

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
) -> Response[ChassisMasterSettingsSaveResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisSettingsSave,
) -> Response[ChassisMasterSettingsSaveResponse200 | ErrorEnvelope]:
    """Chassis master settings save

     The complete REST JSON request is limited to one MiB including both snapshots and JSON escaping.
    Compare expected with a fresh master-settings read, preserve omitted keys, save and reread the same
    source. Existing string fields must remain strings. Parameter ranges and runtime application are not
    established. Local writes serialize; external writers can race. saved means acknowledgment;
    readback_matches verifies storage only. settings_readback_failed means save acknowledged, subsequent
    read failed: reload before retrying.

    Args:
        body (ChassisSettingsSave): Conditional replacement; omitted object keys are retained
            recursively.
            REST limits the complete JSON request, including both snapshots, to one MiB.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMasterSettingsSaveResponse200 | ErrorEnvelope]
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
    body: ChassisSettingsSave,
) -> ChassisMasterSettingsSaveResponse200 | ErrorEnvelope | None:
    """Chassis master settings save

     The complete REST JSON request is limited to one MiB including both snapshots and JSON escaping.
    Compare expected with a fresh master-settings read, preserve omitted keys, save and reread the same
    source. Existing string fields must remain strings. Parameter ranges and runtime application are not
    established. Local writes serialize; external writers can race. saved means acknowledgment;
    readback_matches verifies storage only. settings_readback_failed means save acknowledged, subsequent
    read failed: reload before retrying.

    Args:
        body (ChassisSettingsSave): Conditional replacement; omitted object keys are retained
            recursively.
            REST limits the complete JSON request, including both snapshots, to one MiB.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMasterSettingsSaveResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisSettingsSave,
) -> Response[ChassisMasterSettingsSaveResponse200 | ErrorEnvelope]:
    """Chassis master settings save

     The complete REST JSON request is limited to one MiB including both snapshots and JSON escaping.
    Compare expected with a fresh master-settings read, preserve omitted keys, save and reread the same
    source. Existing string fields must remain strings. Parameter ranges and runtime application are not
    established. Local writes serialize; external writers can race. saved means acknowledgment;
    readback_matches verifies storage only. settings_readback_failed means save acknowledged, subsequent
    read failed: reload before retrying.

    Args:
        body (ChassisSettingsSave): Conditional replacement; omitted object keys are retained
            recursively.
            REST limits the complete JSON request, including both snapshots, to one MiB.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMasterSettingsSaveResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisSettingsSave,
) -> ChassisMasterSettingsSaveResponse200 | ErrorEnvelope | None:
    """Chassis master settings save

     The complete REST JSON request is limited to one MiB including both snapshots and JSON escaping.
    Compare expected with a fresh master-settings read, preserve omitted keys, save and reread the same
    source. Existing string fields must remain strings. Parameter ranges and runtime application are not
    established. Local writes serialize; external writers can race. saved means acknowledgment;
    readback_matches verifies storage only. settings_readback_failed means save acknowledged, subsequent
    read failed: reload before retrying.

    Args:
        body (ChassisSettingsSave): Conditional replacement; omitted object keys are retained
            recursively.
            REST limits the complete JSON request, including both snapshots, to one MiB.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMasterSettingsSaveResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
